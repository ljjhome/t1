// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cyber/proto/clock.proto

#include "cyber/proto/clock.pb.h"

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
namespace cyber {
namespace proto {
constexpr Clock::Clock(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : clock_(uint64_t{0u}){}
struct ClockDefaultTypeInternal {
  constexpr ClockDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~ClockDefaultTypeInternal() {}
  union {
    Clock _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT ClockDefaultTypeInternal _Clock_default_instance_;
}  // namespace proto
}  // namespace cyber
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_cyber_2fproto_2fclock_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_cyber_2fproto_2fclock_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_cyber_2fproto_2fclock_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_cyber_2fproto_2fclock_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::cyber::proto::Clock, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::cyber::proto::Clock, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::cyber::proto::Clock, clock_),
  0,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, -1, sizeof(::apollo::cyber::proto::Clock)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::cyber::proto::_Clock_default_instance_),
};

const char descriptor_table_protodef_cyber_2fproto_2fclock_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\027cyber/proto/clock.proto\022\022apollo.cyber."
  "proto\"\026\n\005Clock\022\r\n\005clock\030\001 \002(\004"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_cyber_2fproto_2fclock_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_cyber_2fproto_2fclock_2eproto = {
  false, false, 69, descriptor_table_protodef_cyber_2fproto_2fclock_2eproto, "cyber/proto/clock.proto", 
  &descriptor_table_cyber_2fproto_2fclock_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_cyber_2fproto_2fclock_2eproto::offsets,
  file_level_metadata_cyber_2fproto_2fclock_2eproto, file_level_enum_descriptors_cyber_2fproto_2fclock_2eproto, file_level_service_descriptors_cyber_2fproto_2fclock_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_cyber_2fproto_2fclock_2eproto_getter() {
  return &descriptor_table_cyber_2fproto_2fclock_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_cyber_2fproto_2fclock_2eproto(&descriptor_table_cyber_2fproto_2fclock_2eproto);
namespace apollo {
namespace cyber {
namespace proto {

// ===================================================================

class Clock::_Internal {
 public:
  using HasBits = decltype(std::declval<Clock>()._has_bits_);
  static void set_has_clock(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static bool MissingRequiredFields(const HasBits& has_bits) {
    return ((has_bits[0] & 0x00000001) ^ 0x00000001) != 0;
  }
};

Clock::Clock(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.cyber.proto.Clock)
}
Clock::Clock(const Clock& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  clock_ = from.clock_;
  // @@protoc_insertion_point(copy_constructor:apollo.cyber.proto.Clock)
}

void Clock::SharedCtor() {
clock_ = uint64_t{0u};
}

Clock::~Clock() {
  // @@protoc_insertion_point(destructor:apollo.cyber.proto.Clock)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Clock::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void Clock::ArenaDtor(void* object) {
  Clock* _this = reinterpret_cast< Clock* >(object);
  (void)_this;
}
void Clock::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Clock::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Clock::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.cyber.proto.Clock)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  clock_ = uint64_t{0u};
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Clock::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // required uint64 clock = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          _Internal::set_has_clock(&has_bits);
          clock_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* Clock::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.cyber.proto.Clock)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required uint64 clock = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt64ToArray(1, this->_internal_clock(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.cyber.proto.Clock)
  return target;
}

size_t Clock::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.cyber.proto.Clock)
  size_t total_size = 0;

  // required uint64 clock = 1;
  if (_internal_has_clock()) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt64SizePlusOne(this->_internal_clock());
  }
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Clock::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Clock::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Clock::GetClassData() const { return &_class_data_; }

void Clock::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Clock *>(to)->MergeFrom(
      static_cast<const Clock &>(from));
}


void Clock::MergeFrom(const Clock& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.cyber.proto.Clock)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_clock()) {
    _internal_set_clock(from._internal_clock());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Clock::CopyFrom(const Clock& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.cyber.proto.Clock)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Clock::IsInitialized() const {
  if (_Internal::MissingRequiredFields(_has_bits_)) return false;
  return true;
}

void Clock::InternalSwap(Clock* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(clock_, other->clock_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Clock::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_cyber_2fproto_2fclock_2eproto_getter, &descriptor_table_cyber_2fproto_2fclock_2eproto_once,
      file_level_metadata_cyber_2fproto_2fclock_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace proto
}  // namespace cyber
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::cyber::proto::Clock* Arena::CreateMaybeMessage< ::apollo::cyber::proto::Clock >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::cyber::proto::Clock >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
