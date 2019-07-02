#pragma once

#include <memory>
#include <stdexcept>

#include <iostream>

template <class T>
class CircularBuffer
{
public:
  explicit CircularBuffer(size_t size)
    : buf_(std::unique_ptr<T[]>(new T[size])), max_size_(size)
  {
  }

  void put(T item)
  {
    buf_[head_] = item;

    if (full_)
    {
      tail_ = (tail_ + 1) % max_size_;
    }

    head_ = (head_ + 1) % max_size_;

    full_ = head_ == tail_;
  }

  T pop_oldest()
  {
    T val = buf_[tail_];

    full_ = false;

    tail_ = (tail_ + 1) % max_size_;

    return val;
  }

  T get_newest()
  {
    int newest_idx = (tail_ + size() - 1) % max_size_;
    return buf_[newest_idx];
  }

  void reset()
  {
    head_ = tail_;
    full_ = false;
  }

  bool empty() const
  {
    return (!full_ && (head_ == tail_));
  }

  bool full() const
  {
    return full_;
  }

  size_t capacity() const
  {
    return max_size_;
  }

  size_t size() const
  {
    size_t size = max_size_;

    if (!full_)
    {
      if (head_ > tail_)
      {
        size = head_ - tail_;
      }
      else
      {
        size = max_size_ + (head_ - tail_);
      }
    }

    return size;
  }

  T &operator[](int i) const
  {
    if (i >= size())
    {
      throw std::out_of_range("Index out of range of buffer");
    }
    int idx = (tail_ + i) % max_size_;

    return buf_[idx];
  }

private:
  std::unique_ptr<T[]> buf_;
  size_t head_ = 0;
  size_t tail_ = 0;
  const size_t max_size_;
  bool full_ = false;
};
