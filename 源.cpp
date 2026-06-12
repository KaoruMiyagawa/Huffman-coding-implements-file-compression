#include <array>
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <list>
#include <stdexcept>
#include <string>
#include <vector>

// 用哈夫曼树实现文件压缩/解压缩（面向 ASCII/字节流）
//
// 压缩文件格式（.huf）：
// [0..3]   magic "HUF1"
// [4..11]  uint64 原文件字节数 original_size
// [12..(12+256*8-1)] 256个 uint64 频率表 freq[256]
// [..]     uint8  padding_bits（末尾补0位数，0~7）
// [..]     哈夫曼编码后的比特流
//
// 解压时使用 original_size 作为停止条件，避免末尾补位导致多解码。

namespace huf {

    static constexpr char kMagic[4] = { 'H', 'U', 'F', '1' };

    struct Node {
        uint64_t freq = 0;
        int symbol = -1;     // 0..255 为叶子结点，其它为内部结点
        int minSymbol = 256; // 子树中最小的symbol，用于稳定排序（保证可重复构树）
        Node* left = nullptr;
        Node* right = nullptr;
    };

    static void freeTree(Node* root) {
        if (!root) return;
        freeTree(root->left);
        freeTree(root->right);
        delete root;
    }

    static bool isLeaf(const Node* n) {
        return n && !n->left && !n->right && n->symbol >= 0;
    }

    // 有序链表：按 (freq, minSymbol) 升序
    static void insertSorted(std::list<Node*>& lst, Node* node) {
        auto it = lst.begin();
        for (; it != lst.end(); ++it) {
            Node* cur = *it;
            if (node->freq < cur->freq) break;
            if (node->freq == cur->freq && node->minSymbol < cur->minSymbol) break;
        }
        lst.insert(it, node);
    }

    static Node* buildTreeFromFreq(const std::array<uint64_t, 256>& freq) {
        std::list<Node*> lst;
        for (int s = 0; s < 256; ++s) {
            if (freq[s] == 0) continue;
            Node* n = new Node();
            n->freq = freq[s];
            n->symbol = s;
            n->minSymbol = s;
            insertSorted(lst, n);
        }

        if (lst.empty()) return nullptr;
        if (lst.size() == 1) return lst.front();

        while (lst.size() > 1) {
            Node* a = lst.front();
            lst.pop_front();
            Node* b = lst.front();
            lst.pop_front();

            Node* parent = new Node();
            parent->freq = a->freq + b->freq;
            parent->left = a;
            parent->right = b;
            parent->minSymbol = std::min(a->minSymbol, b->minSymbol);
            insertSorted(lst, parent);
        }
        return lst.front();
    }

    static void buildCodeTable(Node* root, std::array<std::string, 256>& code, std::string path) {
        if (!root) return;
        if (isLeaf(root)) {
            // 单字符文件时，path可能为空；此时约定编码为"0"
            code[root->symbol] = path.empty() ? "0" : path;
            return;
        }
        if (root->left) buildCodeTable(root->left, code, path + "0");
        if (root->right) buildCodeTable(root->right, code, path + "1");
    }

    class BitWriter {
    public:
        explicit BitWriter(std::ostream& os) : os_(os) {}

        void writeBit(bool bit) {
            cur_ = static_cast<uint8_t>((cur_ << 1) | (bit ? 1 : 0));
            ++bitCount_;
            ++totalBits_;
            if (bitCount_ == 8) flushByte();
        }

        void writeBits01(const std::string& bits01) {
            for (char ch : bits01) writeBit(ch == '1');
        }

        // 返回补0位数（0~7）
        uint8_t finish() {
            if (bitCount_ == 0) return 0;
            uint8_t pad = static_cast<uint8_t>(8 - bitCount_);
            cur_ <<= pad;
            flushByte();
            return pad;
        }

        uint64_t totalBits() const { return totalBits_; }

    private:
        void flushByte() {
            os_.write(reinterpret_cast<const char*>(&cur_), 1);
            cur_ = 0;
            bitCount_ = 0;
        }

        std::ostream& os_;
        uint8_t cur_ = 0;
        uint8_t bitCount_ = 0;
        uint64_t totalBits_ = 0;
    };

    class BitReader {
    public:
        explicit BitReader(std::istream& is) : is_(is) {}

        // 返回：是否成功读到1bit
        bool readBit(bool& outBit) {
            if (remain_ == 0) {
                char c;
                if (!is_.get(c)) return false;
                cur_ = static_cast<uint8_t>(c);
                remain_ = 8;
            }
            outBit = ((cur_ & 0x80) != 0);
            cur_ <<= 1;
            --remain_;
            return true;
        }

    private:
        std::istream& is_;
        uint8_t cur_ = 0;
        uint8_t remain_ = 0;
    };

    static uint64_t fileSize(std::ifstream& ifs) {
        auto pos = ifs.tellg();
        ifs.seekg(0, std::ios::end);
        auto end = ifs.tellg();
        ifs.seekg(pos);
        return static_cast<uint64_t>(end);
    }

    static void writeHeader(std::ofstream& ofs,
        uint64_t originalSize,
        const std::array<uint64_t, 256>& freq,
        uint8_t paddingBits) {
        ofs.write(kMagic, 4);
        ofs.write(reinterpret_cast<const char*>(&originalSize), sizeof(originalSize));
        for (int i = 0; i < 256; ++i) {
            ofs.write(reinterpret_cast<const char*>(&freq[i]), sizeof(uint64_t));
        }
        ofs.write(reinterpret_cast<const char*>(&paddingBits), sizeof(paddingBits));
    }

    struct Header {
        uint64_t originalSize = 0;
        std::array<uint64_t, 256> freq{};
        uint8_t paddingBits = 0;
    };

    static Header readHeader(std::ifstream& ifs) {
        Header h;

        char magic[4];
        if (!ifs.read(magic, 4)) throw std::runtime_error("文件过短：无法读取magic");
        for (int i = 0; i < 4; ++i) {
            if (magic[i] != kMagic[i]) throw std::runtime_error("文件格式错误：magic不匹配");
        }

        if (!ifs.read(reinterpret_cast<char*>(&h.originalSize), sizeof(h.originalSize)))
            throw std::runtime_error("文件过短：无法读取original_size");

        for (int i = 0; i < 256; ++i) {
            if (!ifs.read(reinterpret_cast<char*>(&h.freq[i]), sizeof(uint64_t)))
                throw std::runtime_error("文件过短：无法读取freq表");
        }

        if (!ifs.read(reinterpret_cast<char*>(&h.paddingBits), sizeof(h.paddingBits)))
            throw std::runtime_error("文件过短：无法读取padding_bits");
        if (h.paddingBits > 7) throw std::runtime_error("文件格式错误：padding_bits超范围");

        return h;
    }

    static void compressFile(const std::string& inputPath, const std::string& outputPath) {
        std::ifstream ifs(inputPath, std::ios::binary);
        if (!ifs) throw std::runtime_error("无法打开输入文件：" + inputPath);

        std::array<uint64_t, 256> freq{};
        char ch;
        while (ifs.get(ch)) {
            uint8_t b = static_cast<uint8_t>(ch);
            ++freq[b];
        }

        ifs.clear();
        ifs.seekg(0, std::ios::beg);
        uint64_t originalSize = fileSize(ifs);

        std::ofstream ofs(outputPath, std::ios::binary | std::ios::trunc);
        if (!ofs) throw std::runtime_error("无法创建输出文件：" + outputPath);

        // 先写占位header，paddingBits=0；后面再回填
        writeHeader(ofs, originalSize, freq, 0);
        std::streampos paddingPos = ofs.tellp() - static_cast<std::streamoff>(sizeof(uint8_t));

        Node* root = buildTreeFromFreq(freq);
        std::array<std::string, 256> code{};
        buildCodeTable(root, code, "");

        BitWriter bw(ofs);
        while (ifs.get(ch)) {
            uint8_t b = static_cast<uint8_t>(ch);
            if (!code[b].empty()) bw.writeBits01(code[b]);
        }
        uint8_t padding = bw.finish();

        // 回填 paddingBits
        ofs.seekp(paddingPos);
        ofs.write(reinterpret_cast<const char*>(&padding), sizeof(padding));
        ofs.flush();

        freeTree(root);
    }

    static void decompressFile(const std::string& inputPath, const std::string& outputPath) {
        std::ifstream ifs(inputPath, std::ios::binary);
        if (!ifs) throw std::runtime_error("无法打开输入文件：" + inputPath);

        Header h = readHeader(ifs);

        std::ofstream ofs(outputPath, std::ios::binary | std::ios::trunc);
        if (!ofs) throw std::runtime_error("无法创建输出文件：" + outputPath);

        if (h.originalSize == 0) return; // 空文件

        Node* root = buildTreeFromFreq(h.freq);
        if (!root) throw std::runtime_error("压缩文件损坏：频率表为空但original_size>0");

        // 单字符文件：直接输出 originalSize 次即可（无需读比特流）
        if (isLeaf(root)) {
            char out = static_cast<char>(root->symbol);
            for (uint64_t i = 0; i < h.originalSize; ++i) ofs.put(out);
            freeTree(root);
            return;
        }

        BitReader br(ifs);
        Node* cur = root;
        uint64_t written = 0;
        bool bit = false;

        while (written < h.originalSize) {
            if (!br.readBit(bit)) throw std::runtime_error("压缩文件过短：比特流不足");
            cur = bit ? cur->right : cur->left;
            if (!cur) throw std::runtime_error("压缩文件损坏：解码走到空指针");
            if (isLeaf(cur)) {
                ofs.put(static_cast<char>(cur->symbol));
                ++written;
                cur = root;
            }
        }

        freeTree(root);
    }

    static void printUsage(const char* exe) {
        std::cout
            << "用法：\n"
            << "  " << exe << " c <输入文件> <输出文件.huf>   压缩\n"
            << "  " << exe << " d <输入文件.huf> <输出文件>   解压\n"
            << "也可以直接运行后按提示输入（交互模式）。\n";
    }

    static int runInteractive() {
        while (true) {
            std::cout << "\n=====================================\n";
            std::cout << "  用哈夫曼编码实现文件压缩\n";
            std::cout << "=====================================\n";
            std::cout << "1. 压缩\n";
            std::cout << "2. 解压\n";
            std::cout << "0. 退出\n";
            std::cout << "请选择(0-2)：";
            int op = -1;
            if (!(std::cin >> op)) return 1;
            if (op == 0) return 0;

            std::string in, out;
            std::cout << "输入文件路径：";
            std::cin >> in;
            std::cout << "输出文件路径：";
            std::cin >> out;

            try {
                if (op == 1) {
                    compressFile(in, out);
                    std::cout << "压缩完成。\n";
                }
                else if (op == 2) {
                    decompressFile(in, out);
                    std::cout << "解压完成。\n";
                }
                else {
                    std::cout << "输入不合法，请重试。\n";
                }
            }
            catch (const std::exception& e) {
                std::cout << "失败：" << e.what() << "\n";
            }
        }
    }

} // namespace huf

int main(int argc, char** argv) {
    try {
        if (argc == 1) return huf::runInteractive();
        if (argc != 4) {
            huf::printUsage(argv[0]);
            return 2;
        }

        std::string mode = argv[1];
        if (mode == "c" || mode == "C") {
            huf::compressFile(argv[2], argv[3]);
            return 0;
        }
        if (mode == "d" || mode == "D") {
            huf::decompressFile(argv[2], argv[3]);
            return 0;
        }

        huf::printUsage(argv[0]);
        return 2;
    }
    catch (const std::exception& e) {
        std::cerr << "错误：" << e.what() << "\n";
        return 1;
    }
}
