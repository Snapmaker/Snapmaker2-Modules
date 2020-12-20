/*
 * Snapmaker2-Modules Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Modules
 * (see https://github.com/Snapmaker/Snapmaker2-Modules)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_UTILS_RINGBUFFER_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_UTILS_RINGBUFFER_H_

#include <stdint.h>
#include <stdio.h>
#define DEFAULT_RING_BUFFER_SIZE 128



template <typename T>
class RingBuffer {
 public:
  RingBuffer();
  explicit RingBuffer(int size);
  ~RingBuffer();

  bool isFull();
  bool isEmpty();

  bool insert(const T& element);
  T& remove();
  T& peek();


 private:
  int32_t size;
  int32_t head;
  int32_t tail;
  T * data;

  void init(int size);
  void deinit();
};



template<typename T>
RingBuffer<T>::RingBuffer() {
  init(DEFAULT_RING_BUFFER_SIZE);
}

template<typename T>
RingBuffer<T>::RingBuffer(int size) {
  init(size);
}

template<typename T>
void RingBuffer<T>::init(int size) {
  this->size = size;
  this->head = 0;
  this->tail = 0;
  this->data = new T[size];
}
template<typename T>
void RingBuffer<T>::deinit() {
  this->size = 0;
  this->head = 0;
  this->tail = 0;
  
  if (data != NULL)
    delete data;
  data = NULL;
}
template<typename T>
RingBuffer<T>::~RingBuffer() {
  deinit();
}
template<typename T>
bool RingBuffer<T>::isFull() {
  return (tail + 1 == head) || ((tail + 1) == size && head == 0);
}
template<typename T>
bool RingBuffer<T>::isEmpty() {
  return head == tail;
}
template<typename T>
bool RingBuffer<T>::insert(const T& element) {
  if (isFull()) {
    return false;
  }
  data[tail] = element;
  tail = (tail + 1 == size) ? 0 : tail + 1;
  return true;
}
template<typename T>
T& RingBuffer<T>::remove() {
  if (isEmpty()) {
    // safe guard?
    // return nullptr;
  }
  T& ret = data[head];
  head = (head + 1 == size) ? 0 : head + 1;

  return ret;
}

template<typename T>
T& RingBuffer<T>::peek() {
  if (isEmpty()) {
    // return nullptr;
  }
  return data[head];
}


#endif //MODULES_WHIMSYCWD_MARLIN_SRC_UTILS_RINGBUFFER_H_
