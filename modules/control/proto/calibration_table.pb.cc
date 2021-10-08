// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/control/proto/calibration_table.proto

#include "modules/control/proto/calibration_table.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG
namespace apollo {
namespace control {
namespace calibrationtable {
constexpr ControlCalibrationTable::ControlCalibrationTable(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : calibration_(){}
struct ControlCalibrationTableDefaultTypeInternal {
  constexpr ControlCalibrationTableDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~ControlCalibrationTableDefaultTypeInternal() {}
  union {
    ControlCalibrationTable _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT ControlCalibrationTableDefaultTypeInternal _ControlCalibrationTable_default_instance_;
constexpr ControlCalibrationInfo::ControlCalibrationInfo(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : speed_(0)
  , acceleration_(0)
  , command_(0){}
struct ControlCalibrationInfoDefaultTypeInternal {
  constexpr ControlCalibrationInfoDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~ControlCalibrationInfoDefaultTypeInternal() {}
  union {
    ControlCalibrationInfo _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT ControlCalibrationInfoDefaultTypeInternal _ControlCalibrationInfo_default_instance_;
}  // namespace calibrationtable
}  // namespace control
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::apollo::control::calibrationtable::ControlCalibrationTable, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::control::calibrationtable::ControlCalibrationTable, calibration_),
  PROTOBUF_FIELD_OFFSET(::apollo::control::calibrationtable::ControlCalibrationInfo, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::control::calibrationtable::ControlCalibrationInfo, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::control::calibrationtable::ControlCalibrationInfo, speed_),
  PROTOBUF_FIELD_OFFSET(::apollo::control::calibrationtable::ControlCalibrationInfo, acceleration_),
  PROTOBUF_FIELD_OFFSET(::apollo::control::calibrationtable::ControlCalibrationInfo, command_),
  0,
  1,
  2,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::apollo::control::calibrationtable::ControlCalibrationTable)},
  { 7, 16, -1, sizeof(::apollo::control::calibrationtable::ControlCalibrationInfo)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::control::calibrationtable::_ControlCalibrationTable_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::control::calibrationtable::_ControlCalibrationInfo_default_instance_),
};

const char descriptor_table_protodef_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n-modules/control/proto/calibration_tabl"
  "e.proto\022\037apollo.control.calibrationtable"
  "\"g\n\027ControlCalibrationTable\022L\n\013calibrati"
  "on\030\001 \003(\01327.apollo.control.calibrationtab"
  "le.ControlCalibrationInfo\"N\n\026ControlCali"
  "brationInfo\022\r\n\005speed\030\001 \001(\001\022\024\n\014accelerati"
  "on\030\002 \001(\001\022\017\n\007command\030\003 \001(\001"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto = {
  false, false, 265, descriptor_table_protodef_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto, "modules/control/proto/calibration_table.proto", 
  &descriptor_table_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto_once, nullptr, 0, 2,
  schemas, file_default_instances, TableStruct_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto::offsets,
  file_level_metadata_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto, file_level_enum_descriptors_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto, file_level_service_descriptors_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto_getter() {
  return &descriptor_table_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto(&descriptor_table_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto);
namespace apollo {
namespace control {
namespace calibrationtable {

// ===================================================================

class ControlCalibrationTable::_Internal {
 public:
};

ControlCalibrationTable::ControlCalibrationTable(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  calibration_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.control.calibrationtable.ControlCalibrationTable)
}
ControlCalibrationTable::ControlCalibrationTable(const ControlCalibrationTable& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      calibration_(from.calibration_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:apollo.control.calibrationtable.ControlCalibrationTable)
}

void ControlCalibrationTable::SharedCtor() {
}

ControlCalibrationTable::~ControlCalibrationTable() {
  // @@protoc_insertion_point(destructor:apollo.control.calibrationtable.ControlCalibrationTable)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void ControlCalibrationTable::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void ControlCalibrationTable::ArenaDtor(void* object) {
  ControlCalibrationTable* _this = reinterpret_cast< ControlCalibrationTable* >(object);
  (void)_this;
}
void ControlCalibrationTable::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void ControlCalibrationTable::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void ControlCalibrationTable::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.control.calibrationtable.ControlCalibrationTable)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  calibration_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* ControlCalibrationTable::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // repeated .apollo.control.calibrationtable.ControlCalibrationInfo calibration = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_calibration(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<10>(ptr));
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* ControlCalibrationTable::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.control.calibrationtable.ControlCalibrationTable)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .apollo.control.calibrationtable.ControlCalibrationInfo calibration = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_calibration_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, this->_internal_calibration(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.control.calibrationtable.ControlCalibrationTable)
  return target;
}

size_t ControlCalibrationTable::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.control.calibrationtable.ControlCalibrationTable)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .apollo.control.calibrationtable.ControlCalibrationInfo calibration = 1;
  total_size += 1UL * this->_internal_calibration_size();
  for (const auto& msg : this->calibration_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData ControlCalibrationTable::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    ControlCalibrationTable::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*ControlCalibrationTable::GetClassData() const { return &_class_data_; }

void ControlCalibrationTable::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<ControlCalibrationTable *>(to)->MergeFrom(
      static_cast<const ControlCalibrationTable &>(from));
}


void ControlCalibrationTable::MergeFrom(const ControlCalibrationTable& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.control.calibrationtable.ControlCalibrationTable)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  calibration_.MergeFrom(from.calibration_);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void ControlCalibrationTable::CopyFrom(const ControlCalibrationTable& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.control.calibrationtable.ControlCalibrationTable)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ControlCalibrationTable::IsInitialized() const {
  return true;
}

void ControlCalibrationTable::InternalSwap(ControlCalibrationTable* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  calibration_.InternalSwap(&other->calibration_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ControlCalibrationTable::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto_getter, &descriptor_table_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto_once,
      file_level_metadata_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto[0]);
}

// ===================================================================

class ControlCalibrationInfo::_Internal {
 public:
  using HasBits = decltype(std::declval<ControlCalibrationInfo>()._has_bits_);
  static void set_has_speed(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_acceleration(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_command(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

ControlCalibrationInfo::ControlCalibrationInfo(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.control.calibrationtable.ControlCalibrationInfo)
}
ControlCalibrationInfo::ControlCalibrationInfo(const ControlCalibrationInfo& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&speed_, &from.speed_,
    static_cast<size_t>(reinterpret_cast<char*>(&command_) -
    reinterpret_cast<char*>(&speed_)) + sizeof(command_));
  // @@protoc_insertion_point(copy_constructor:apollo.control.calibrationtable.ControlCalibrationInfo)
}

void ControlCalibrationInfo::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&speed_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&command_) -
    reinterpret_cast<char*>(&speed_)) + sizeof(command_));
}

ControlCalibrationInfo::~ControlCalibrationInfo() {
  // @@protoc_insertion_point(destructor:apollo.control.calibrationtable.ControlCalibrationInfo)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void ControlCalibrationInfo::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void ControlCalibrationInfo::ArenaDtor(void* object) {
  ControlCalibrationInfo* _this = reinterpret_cast< ControlCalibrationInfo* >(object);
  (void)_this;
}
void ControlCalibrationInfo::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void ControlCalibrationInfo::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void ControlCalibrationInfo::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.control.calibrationtable.ControlCalibrationInfo)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    ::memset(&speed_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&command_) -
        reinterpret_cast<char*>(&speed_)) + sizeof(command_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* ControlCalibrationInfo::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional double speed = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          _Internal::set_has_speed(&has_bits);
          speed_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional double acceleration = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 17)) {
          _Internal::set_has_acceleration(&has_bits);
          acceleration_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional double command = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 25)) {
          _Internal::set_has_command(&has_bits);
          command_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* ControlCalibrationInfo::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.control.calibrationtable.ControlCalibrationInfo)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double speed = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_speed(), target);
  }

  // optional double acceleration = 2;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_acceleration(), target);
  }

  // optional double command = 3;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(3, this->_internal_command(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.control.calibrationtable.ControlCalibrationInfo)
  return target;
}

size_t ControlCalibrationInfo::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.control.calibrationtable.ControlCalibrationInfo)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional double speed = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 + 8;
    }

    // optional double acceleration = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 + 8;
    }

    // optional double command = 3;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 + 8;
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData ControlCalibrationInfo::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    ControlCalibrationInfo::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*ControlCalibrationInfo::GetClassData() const { return &_class_data_; }

void ControlCalibrationInfo::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<ControlCalibrationInfo *>(to)->MergeFrom(
      static_cast<const ControlCalibrationInfo &>(from));
}


void ControlCalibrationInfo::MergeFrom(const ControlCalibrationInfo& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.control.calibrationtable.ControlCalibrationInfo)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      speed_ = from.speed_;
    }
    if (cached_has_bits & 0x00000002u) {
      acceleration_ = from.acceleration_;
    }
    if (cached_has_bits & 0x00000004u) {
      command_ = from.command_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void ControlCalibrationInfo::CopyFrom(const ControlCalibrationInfo& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.control.calibrationtable.ControlCalibrationInfo)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ControlCalibrationInfo::IsInitialized() const {
  return true;
}

void ControlCalibrationInfo::InternalSwap(ControlCalibrationInfo* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(ControlCalibrationInfo, command_)
      + sizeof(ControlCalibrationInfo::command_)
      - PROTOBUF_FIELD_OFFSET(ControlCalibrationInfo, speed_)>(
          reinterpret_cast<char*>(&speed_),
          reinterpret_cast<char*>(&other->speed_));
}

::PROTOBUF_NAMESPACE_ID::Metadata ControlCalibrationInfo::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto_getter, &descriptor_table_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto_once,
      file_level_metadata_modules_2fcontrol_2fproto_2fcalibration_5ftable_2eproto[1]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace calibrationtable
}  // namespace control
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::control::calibrationtable::ControlCalibrationTable* Arena::CreateMaybeMessage< ::apollo::control::calibrationtable::ControlCalibrationTable >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::control::calibrationtable::ControlCalibrationTable >(arena);
}
template<> PROTOBUF_NOINLINE ::apollo::control::calibrationtable::ControlCalibrationInfo* Arena::CreateMaybeMessage< ::apollo::control::calibrationtable::ControlCalibrationInfo >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::control::calibrationtable::ControlCalibrationInfo >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
