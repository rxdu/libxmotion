/*
 * @file utest_ring_buffer.cpp
 * @date 10/10/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include "async_port/ring_buffer.hpp"

using namespace xmotion;

class RingBufferTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Code here will be called immediately after the constructor (right before
    // each test).
  }

  void TearDown() override {
    // Code here will be called immediately after each test (right before the
    // destructor).
  }

  // Objects declared here can be used by all tests in the test case.
  RingBuffer<int, 8> buffer_no_overwrite{false};
  RingBuffer<int, 8> buffer_with_overwrite{true};
};

TEST_F(RingBufferTest, IsEmptyInitially) {
  EXPECT_TRUE(buffer_no_overwrite.IsEmpty());
  EXPECT_TRUE(buffer_with_overwrite.IsEmpty());
}

TEST_F(RingBufferTest, IsNotFullInitially) {
  EXPECT_FALSE(buffer_no_overwrite.IsFull());
  EXPECT_FALSE(buffer_with_overwrite.IsFull());
}

TEST_F(RingBufferTest, CanWriteAndRead) {
  buffer_no_overwrite.Write(1);
  buffer_with_overwrite.Write(2);

  int value;
  buffer_no_overwrite.Read(value);
  EXPECT_EQ(value, 1);

  buffer_with_overwrite.Read(value);
  EXPECT_EQ(value, 2);

  EXPECT_TRUE(buffer_no_overwrite.IsEmpty());
  EXPECT_TRUE(buffer_with_overwrite.IsEmpty());
}

TEST_F(RingBufferTest, PeekData) {
  buffer_no_overwrite.Reset();
  EXPECT_TRUE(buffer_no_overwrite.IsEmpty());
  buffer_no_overwrite.Write(1);
  buffer_no_overwrite.Write(3);
  EXPECT_EQ(buffer_no_overwrite.GetOccupiedSize(), 2);

  std::vector<int> value(7);
  buffer_no_overwrite.Peek(value, 1);
  EXPECT_EQ(value[0], 1);
  buffer_no_overwrite.Peek(value, 2);
  EXPECT_EQ(value[0], 1);
  EXPECT_EQ(value[1], 3);
  EXPECT_EQ(buffer_no_overwrite.GetOccupiedSize(), 2);

  // Test with overwrite enabled
  buffer_with_overwrite.Reset();
  EXPECT_TRUE(buffer_with_overwrite.IsEmpty());
  buffer_with_overwrite.Write(1);
  buffer_with_overwrite.Write(3);
  EXPECT_EQ(buffer_with_overwrite.GetOccupiedSize(), 2);

  buffer_with_overwrite.Peek(value, 1);
  EXPECT_EQ(value[0], 1);
  buffer_with_overwrite.Peek(value, 2);
  EXPECT_EQ(value[0], 1);
  EXPECT_EQ(value[1], 3);
  EXPECT_EQ(buffer_with_overwrite.GetOccupiedSize(), 2);
}

TEST_F(RingBufferTest, ReadWriteMultiple) {
  std::vector<int> input = {1, 2, 3, 4, 5, 6};
  std::vector<int> output(6);

  buffer_no_overwrite.Reset();
  buffer_no_overwrite.Write(input, 6);
  buffer_no_overwrite.Read(output, 6);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(input[i], output[i]);
  }

  buffer_with_overwrite.Reset();
  buffer_with_overwrite.Write(input, 6);
  buffer_with_overwrite.Read(output, 6);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(input[i], output[i]);
  }
}

TEST_F(RingBufferTest, MaintainsOrder) {
  buffer_no_overwrite.Write(1);
  buffer_no_overwrite.Write(2);
  buffer_no_overwrite.Write(3);

  buffer_with_overwrite.Write(4);
  buffer_with_overwrite.Write(5);
  buffer_with_overwrite.Write(6);

  int value;
  buffer_no_overwrite.Read(value);
  EXPECT_EQ(value, 1);
  buffer_no_overwrite.Read(value);
  EXPECT_EQ(value, 2);
  buffer_no_overwrite.Read(value);
  EXPECT_EQ(value, 3);

  buffer_with_overwrite.Read(value);
  EXPECT_EQ(value, 4);
  buffer_with_overwrite.Read(value);
  EXPECT_EQ(value, 5);
  buffer_with_overwrite.Read(value);
  EXPECT_EQ(value, 6);
}

TEST_F(RingBufferTest, OverwritesWhenFull) {
  buffer_no_overwrite.Reset();
  buffer_with_overwrite.Reset();

  // the buffer can only hold 8 - 1 = 7 elements
  for (int i = 0; i < 7; ++i) {
    buffer_no_overwrite.Write(i);
    buffer_with_overwrite.Write(i);
  }
  ASSERT_TRUE(buffer_no_overwrite.IsFull());
  ASSERT_TRUE(buffer_with_overwrite.IsFull());

  buffer_no_overwrite.Write(8);  // This should not overwrite the first element
  int value;
  buffer_no_overwrite.Read(value);
  EXPECT_EQ(value, 0);
  ASSERT_FALSE(buffer_no_overwrite.IsFull());
  EXPECT_EQ(buffer_no_overwrite.GetOccupiedSize(), 6);
  EXPECT_EQ(buffer_no_overwrite.GetFreeSize(), 1);

  buffer_with_overwrite.Write(8);  // This should overwrite the first element
  ASSERT_TRUE(buffer_with_overwrite.IsFull());

  buffer_with_overwrite.Read(value);
  EXPECT_EQ(value, 1);  // The first element (0) should be overwritten
  ASSERT_FALSE(buffer_no_overwrite.IsFull());
  EXPECT_EQ(buffer_no_overwrite.GetOccupiedSize(), 6);
  EXPECT_EQ(buffer_no_overwrite.GetFreeSize(), 1);
}

TEST_F(RingBufferTest, HandlesWrapAround) {
  buffer_no_overwrite.Reset();
  int value;
  for (int i = 0; i < 15; ++i) {
    buffer_no_overwrite.Write(i);
    buffer_no_overwrite.Read(value);
    EXPECT_EQ(value, i);
    EXPECT_EQ(buffer_no_overwrite.GetFreeSize(), 7);
  }

  buffer_with_overwrite.Reset();
  for (int i = 0; i < 15; ++i) {
    buffer_with_overwrite.Write(i);
    buffer_with_overwrite.Read(value);
    EXPECT_EQ(value, i);
    EXPECT_EQ(buffer_with_overwrite.GetFreeSize(), 7);
  }
}
