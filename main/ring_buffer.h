#include <cstddef>
#include <array>
#include <utility>

template <typename T, size_t N>
class RingBuffer
{
public:
    explicit RingBuffer(
        const bool overwrite_on_full = false)
        : overwrite_on_full_(overwrite_on_full)
    {
    }

    bool Write(const T& data)
    {
        if (this->IsFull())
        {
            if (!this->overwrite_on_full_)
            {
                return false;
            }

            // 가장 오래된 데이터 폐기
            this->read_index_ =
                (this->read_index_ + 1) % N;
        }
        else
        {
            ++this->count_;
        }

        this->buffer_[this->write_index_] = data;

        this->write_index_ =
            (this->write_index_ + 1) % N;

        return true;
    }

    bool Read(T& out)
    {
        if (this->IsEmpty())
        {
            return false;
        }

        out = this->buffer_[this->read_index_];

        this->read_index_ =
            (this->read_index_ + 1) % N;

        --this->count_;

        return true;
    }

    bool Peek(
        const size_t offset,
        T& out) const
    {
        if (offset >= this->count_)
        {
            return false;
        }

        const size_t index =
            (this->read_index_ + offset) % N;

        out = this->buffer_[index];

        return true;
    }

    bool IsFull() const
    {
        return this->count_ == N;
    }

    bool IsEmpty() const
    {
        return this->count_ == 0;
    }

    size_t Size() const
    {
        return this->count_;
    }

private:
    T buffer_[N]{};

    size_t write_index_ = 0;
    size_t read_index_ = 0;
    size_t count_ = 0;

    bool overwrite_on_full_ = false;

    static_assert(N > 0, "RingBuffer size must be greater than zero.");
};

