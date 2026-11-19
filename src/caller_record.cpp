// CallerRecord: per-task outcome records of the shared port objects, in thread-local storage, so a
// task never reads another task's failure. Platform-free. Implements CallerRecord
// (kb/port/esp32/error.hpp), used by the sources of gpio, i2c, uart, spi, adc, pwm and can
// (src/<header>.cpp). SPDX-License-Identifier: MIT Copyright (c) 2026 The port-esp32 authors
#include <array>
// Lock-free claims, counters and flags shared between tasks (deviations/deviations.yml, DEV-003).
// misra-lint: ignore KB-CPP-012
#include <atomic>
#include <cstdint>

#include "kb/port/esp32/error.hpp"
#include "kb/utils/result.hpp"
#include "kb/utils/status.hpp"

namespace kb::port::esp32 {

namespace {
struct Entry {
  uint32_t owner{0U}; // 0: unused
  uint32_t stamp{0U};
  uint32_t detail{0U};
  kb::Status status{kb::Status::Ok};
  uint8_t aux{0U};
};

struct Records {
  std::array<Entry, kCallerRecordSlots> entries{};
  uint32_t clock{0U};
};

Records& Mine() noexcept {
  thread_local Records records{};
  return records;
}

uint32_t NextId() noexcept {
  // codeql::misra_deviation_next_line(DEV-003)
  static std::atomic<uint32_t> next{1U};
  uint32_t id = next.fetch_add(1U, std::memory_order_relaxed);
  if (id == 0U) {
    id = next.fetch_add(1U, std::memory_order_relaxed); // 0 marks an unused entry
  }
  return id;
}

const Entry* Find(const Records& records, uint32_t owner) noexcept {
  const Entry* found = nullptr;
  for (const Entry& e : records.entries) {
    if (e.owner == owner) {
      found = &e;
    }
  }
  return found;
}
} // namespace

CallerRecord::CallerRecord() noexcept : id_(NextId()) {}

kb::Status CallerRecord::Set(kb::Status status, int32_t err, uint8_t aux) const noexcept {
  Records& records = Mine();
  Entry* slot = records.entries.data();
  for (Entry& e : records.entries) {
    if (e.owner == id_) {
      slot = &e;
      break;
    }
    if (e.stamp < slot->stamp) {
      slot = &e; // the oldest, unless this object already has one
    }
  }
  ++records.clock;
  slot->owner = id_;
  slot->stamp = records.clock;
  slot->status = status;
  slot->detail = VendorCode(err);
  slot->aux = aux;
  return status;
}

kb::Result<void> CallerRecord::Get() const noexcept {
  const Entry* e = Find(Mine(), id_);
  return ((e == nullptr) || (e->status == kb::Status::Ok))
             ? kb::Result<void>::Success()
             : kb::Result<void>::Failure(e->status, e->detail);
}

uint8_t CallerRecord::Aux() const noexcept {
  const Entry* e = Find(Mine(), id_);
  return (e == nullptr) ? uint8_t{0U} : e->aux;
}

} // namespace kb::port::esp32
