/**
 * @file error.hpp
 * @brief The ESP-IDF error codes this port reports, and their mapping to kb::Status.
 *
 * Every failed SDK call is reported as a kb::Status, and the `esp_err_t` it returned is kept as the
 * vendor code: in `Detail()` of a kb::Result, and in `LastError().Detail()` of the object for calls
 * that return a bare kb::Status. The values below are the ESP-IDF v5.5 `esp_err.h` values; each
 * `src/boundary/error.cpp` checks them against the SDK with a static_assert, so a change in the SDK
 * stops the build.
 *
 * | esp_err_t | Value | kb::Status |
 * | --- | --- | --- |
 * | ESP_OK | 0 | Ok |
 * | ESP_FAIL | -1 | Fault |
 * | ESP_ERR_NO_MEM | 0x101 | Fault |
 * | ESP_ERR_INVALID_ARG | 0x102 | InvalidArgument |
 * | ESP_ERR_INVALID_STATE | 0x103 | NotReady |
 * | ESP_ERR_INVALID_SIZE | 0x104 | OutOfRange |
 * | ESP_ERR_NOT_FOUND | 0x105 | NoDevice |
 * | ESP_ERR_NOT_SUPPORTED | 0x106 | Unsupported |
 * | ESP_ERR_TIMEOUT | 0x107 | Timeout |
 * | ESP_ERR_INVALID_RESPONSE | 0x108 | BusError |
 * | ESP_ERR_INVALID_CRC | 0x109 | CrcMismatch |
 * | ESP_ERR_NOT_FINISHED | 0x10C | Busy |
 * | ESP_ERR_NOT_ALLOWED | 0x10D | Busy |
 * | anything else | | Fault |
 *
 * A peripheral may refine the general mapping where the SDK overloads a code: the I2C controller
 * turns ESP_ERR_INVALID_STATE after a transfer into NoDevice or Nack (docs/error-mapping.md).
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_ERROR_HPP
#define KB_PORT_ESP32_ERROR_HPP

// Lock-free claims, counters and flags shared between tasks (deviations/deviations.yml, DEV-003).
// misra-lint: ignore KB-CPP-012
#include <atomic>
#include <cstdint>

#include "kb/utils/result.hpp"
#include "kb/utils/status.hpp"
#include "kb/utils/time.hpp"

namespace kb::port::esp32 {

/** @brief The ESP-IDF `esp_err_t` values this port names (checked against the SDK). */
namespace esp_err {
inline constexpr int32_t kOk = 0;                  ///< ESP_OK.
inline constexpr int32_t kFail = -1;               ///< ESP_FAIL.
inline constexpr int32_t kNoMem = 0x101;           ///< ESP_ERR_NO_MEM.
inline constexpr int32_t kInvalidArg = 0x102;      ///< ESP_ERR_INVALID_ARG.
inline constexpr int32_t kInvalidState = 0x103;    ///< ESP_ERR_INVALID_STATE.
inline constexpr int32_t kInvalidSize = 0x104;     ///< ESP_ERR_INVALID_SIZE.
inline constexpr int32_t kNotFound = 0x105;        ///< ESP_ERR_NOT_FOUND.
inline constexpr int32_t kNotSupported = 0x106;    ///< ESP_ERR_NOT_SUPPORTED.
inline constexpr int32_t kTimeout = 0x107;         ///< ESP_ERR_TIMEOUT.
inline constexpr int32_t kInvalidResponse = 0x108; ///< ESP_ERR_INVALID_RESPONSE.
inline constexpr int32_t kInvalidCrc = 0x109;      ///< ESP_ERR_INVALID_CRC.
inline constexpr int32_t kNotFinished = 0x10C;     ///< ESP_ERR_NOT_FINISHED.
inline constexpr int32_t kNotAllowed = 0x10D;      ///< ESP_ERR_NOT_ALLOWED.
} // namespace esp_err

/** @brief The general mapping of an `esp_err_t` to kb::Status (table in the file comment). */
[[nodiscard]] constexpr kb::Status MapEspErr(int32_t err) noexcept {
  kb::Status status = kb::Status::Fault;
  switch (err) {
  case esp_err::kOk:
    status = kb::Status::Ok;
    break;
  case esp_err::kInvalidArg:
    status = kb::Status::InvalidArgument;
    break;
  case esp_err::kInvalidState:
    status = kb::Status::NotReady;
    break;
  case esp_err::kInvalidSize:
    status = kb::Status::OutOfRange;
    break;
  case esp_err::kNotFound:
    status = kb::Status::NoDevice;
    break;
  case esp_err::kNotSupported:
    status = kb::Status::Unsupported;
    break;
  case esp_err::kTimeout:
    status = kb::Status::Timeout;
    break;
  case esp_err::kInvalidResponse:
    status = kb::Status::BusError;
    break;
  case esp_err::kInvalidCrc:
    status = kb::Status::CrcMismatch;
    break;
  case esp_err::kNotFinished:
  case esp_err::kNotAllowed:
    status = kb::Status::Busy;
    break;
  default:
    status = kb::Status::Fault;
    break;
  }
  return status;
}

/** @brief The `esp_err_t` as the unsigned vendor code of a kb::Result (two's complement for
 * ESP_FAIL). */
[[nodiscard]] constexpr uint32_t VendorCode(int32_t err) noexcept {
  return static_cast<uint32_t>(err);
}

/**
 * @brief The outcome of the last call of a port object that failed or succeeded: the status and the
 * `esp_err_t` behind it. `Ok` with detail 0 after a success. For objects used from one context at a
 * time (pins, PWM channels); shared objects use CallerRecord.
 */
class LastErrorRecord {
public:
  constexpr LastErrorRecord() noexcept = default;
  /** @brief Records `status` with vendor code `err` and returns `status`. */
  constexpr kb::Status Set(kb::Status status, int32_t err) noexcept {
    status_ = status;
    detail_ = VendorCode(err);
    return status;
  }
  /** @brief Records a success. */
  constexpr kb::Status Clear() noexcept {
    return Set(kb::Status::Ok, esp_err::kOk);
  }
  /** @brief The record as a kb::Result<void>: `Error()` and `Detail()` (the esp_err_t). */
  [[nodiscard]] constexpr kb::Result<void> Get() const noexcept {
    return (status_ == kb::Status::Ok) ? kb::Result<void>::Success()
                                       : kb::Result<void>::Failure(status_, detail_);
  }

private:
  kb::Status status_{kb::Status::Ok};
  uint32_t detail_{0U};
};

/**
 * @brief The longest wait of any port call: 2 s. Every timeout above it, `kb::kMaxMilliseconds`
 * (kb-core's "no limit") included, waits 2 s, so no call of the port can wait for ever.
 */
inline constexpr kb::Milliseconds kMaxTimeout{2000U};

/** @brief `timeout` capped at kMaxTimeout, in milliseconds. */
[[nodiscard]] constexpr uint32_t BoundedMs(kb::Milliseconds timeout) noexcept {
  return (timeout.Count() > kMaxTimeout.Count()) ? kMaxTimeout.Count() : timeout.Count();
}

/**
 * @brief The outcome of the last call of a shared port object, kept per calling task: a task reads
 * only the record of its own last call on this object, never another task's.
 *
 * The records live in thread-local storage (up to kCallerRecordSlots objects per task; the oldest
 * is reused). A task that has not called the object, or whose record was reused, reads `Ok` with
 * aux 0. `aux` carries one extra byte (the I2C failure kind). Not for interrupt context.
 */
class CallerRecord {
public:
  /** @brief A record with a new identity (records of a destroyed object are never matched). */
  CallerRecord() noexcept;
  CallerRecord(const CallerRecord&) = delete;
  CallerRecord& operator=(const CallerRecord&) = delete;
  CallerRecord(CallerRecord&&) = delete;
  CallerRecord& operator=(CallerRecord&&) = delete;
  ~CallerRecord() = default;

  /** @brief Records `status` with vendor code `err` for the calling task and returns `status`. */
  [[nodiscard]] kb::Status Set(kb::Status status, int32_t err, uint8_t aux = 0U) const noexcept;
  /** @brief Records a success. */
  [[nodiscard]] kb::Status Clear() const noexcept {
    return Set(kb::Status::Ok, esp_err::kOk);
  }
  /** @brief The calling task's record as a kb::Result<void>. */
  [[nodiscard]] kb::Result<void> Get() const noexcept;
  /** @brief The calling task's extra byte. */
  [[nodiscard]] uint8_t Aux() const noexcept;

private:
  uint32_t id_;
};

/** @brief Records per task and object kept by each task. */
inline constexpr uint32_t kCallerRecordSlots = 4U;

/** @brief A statistics counter that several tasks may bump (relaxed atomic). */
class Counter {
public:
  constexpr Counter() noexcept = default;
  Counter(const Counter&) = delete;
  Counter& operator=(const Counter&) = delete;
  Counter(Counter&&) = delete;
  Counter& operator=(Counter&&) = delete;
  ~Counter() = default;
  /** @brief Adds `n`. */
  void Add(uint32_t n = 1U) noexcept {
    static_cast<void>(value_.fetch_add(n, std::memory_order_relaxed));
  }
  /** @brief The count. */
  [[nodiscard]] uint32_t Get() const noexcept {
    return value_.load(std::memory_order_relaxed);
  }
  /** @brief Back to 0. */
  void Reset() noexcept {
    value_.store(0U, std::memory_order_relaxed);
  }

private:
  std::atomic<uint32_t> value_{0U};
};

} // namespace kb::port::esp32

#endif // KB_PORT_ESP32_ERROR_HPP
