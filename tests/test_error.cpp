// The esp_err_t mapping (error.hpp) and the last-error record.
// Test code. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include "support.hpp"

namespace e = kb::port::esp32::esp_err;

// Every row of the mapping table, at compile time.
static_assert(esp::MapEspErr(e::kOk) == Status::Ok);
static_assert(esp::MapEspErr(e::kFail) == Status::Fault);
static_assert(esp::MapEspErr(e::kNoMem) == Status::Fault);
static_assert(esp::MapEspErr(e::kInvalidArg) == Status::InvalidArgument);
static_assert(esp::MapEspErr(e::kInvalidState) == Status::NotReady);
static_assert(esp::MapEspErr(e::kInvalidSize) == Status::OutOfRange);
static_assert(esp::MapEspErr(e::kNotFound) == Status::NoDevice);
static_assert(esp::MapEspErr(e::kNotSupported) == Status::Unsupported);
static_assert(esp::MapEspErr(e::kTimeout) == Status::Timeout);
static_assert(esp::MapEspErr(e::kInvalidResponse) == Status::BusError);
static_assert(esp::MapEspErr(e::kInvalidCrc) == Status::CrcMismatch);
static_assert(esp::MapEspErr(e::kNotFinished) == Status::Busy);
static_assert(esp::MapEspErr(e::kNotAllowed) == Status::Busy);
static_assert(esp::MapEspErr(0x10A) == Status::Fault); // ESP_ERR_INVALID_VERSION: not named, Fault
static_assert(esp::VendorCode(e::kFail) == 0xFFFFFFFFU);

KB_TEST("error: the mapping at run time agrees with the table") {
  const std::array<int32_t, 13> codes{
      e::kOk,          e::kFail,        e::kNoMem,        e::kInvalidArg, e::kInvalidState,
      e::kInvalidSize, e::kNotFound,    e::kNotSupported, e::kTimeout,    e::kInvalidResponse,
      e::kInvalidCrc,  e::kNotFinished, e::kNotAllowed};
  const std::array<Status, 13> want{
      Status::Ok,       Status::Fault,      Status::Fault,       Status::InvalidArgument,
      Status::NotReady, Status::OutOfRange, Status::NoDevice,    Status::Unsupported,
      Status::Timeout,  Status::BusError,   Status::CrcMismatch, Status::Busy,
      Status::Busy};
  for (std::size_t i = 0U; i < codes.size(); ++i) {
    KB_CHECK(esp::MapEspErr(codes[i]) == want[i]);
  }
  KB_CHECK(esp::MapEspErr(12345) == Status::Fault);
}

KB_TEST("error: the record keeps the last status and the esp_err_t") {
  esp::LastErrorRecord r{};
  KB_CHECK(r.Get().IsOk());
  KB_CHECK(r.Set(Status::Timeout, e::kTimeout) == Status::Timeout);
  KB_CHECK(FailedWith(r.Get(), Status::Timeout, e::kTimeout));
  KB_CHECK(r.Clear() == Status::Ok);
  KB_CHECK(r.Get().IsOk());
  KB_CHECK_EQ(r.Get().Detail(), 0U);
}
