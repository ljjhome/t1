// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/common/proto/vehicle_signal.proto

#include "modules/common/proto/vehicle_signal.pb.h"

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
namespace common {
constexpr VehicleSignal::VehicleSignal(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : turn_signal_(0)

  , high_beam_(false)
  , low_beam_(false)
  , horn_(false)
  , emergency_light_(false){}
struct VehicleSignalDefaultTypeInternal {
  constexpr VehicleSignalDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~VehicleSignalDefaultTypeInternal() {}
  union {
    VehicleSignal _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT VehicleSignalDefaultTypeInternal _VehicleSignal_default_instance_;
}  // namespace common
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto[1];
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::common::VehicleSignal, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::common::VehicleSignal, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::common::VehicleSignal, turn_signal_),
  PROTOBUF_FIELD_OFFSET(::apollo::common::VehicleSignal, high_beam_),
  PROTOBUF_FIELD_OFFSET(::apollo::common::VehicleSignal, low_beam_),
  PROTOBUF_FIELD_OFFSET(::apollo::common::VehicleSignal, horn_),
  PROTOBUF_FIELD_OFFSET(::apollo::common::VehicleSignal, emergency_light_),
  0,
  1,
  2,
  3,
  4,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 11, -1, sizeof(::apollo::common::VehicleSignal)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::common::_VehicleSignal_default_instance_),
};

const char descriptor_table_protodef_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n)modules/common/proto/vehicle_signal.pr"
  "oto\022\rapollo.common\"\325\001\n\rVehicleSignal\022<\n\013"
  "turn_signal\030\001 \001(\0162\'.apollo.common.Vehicl"
  "eSignal.TurnSignal\022\021\n\thigh_beam\030\002 \001(\010\022\020\n"
  "\010low_beam\030\003 \001(\010\022\014\n\004horn\030\004 \001(\010\022\027\n\017emergen"
  "cy_light\030\005 \001(\010\":\n\nTurnSignal\022\r\n\tTURN_NON"
  "E\020\000\022\r\n\tTURN_LEFT\020\001\022\016\n\nTURN_RIGHT\020\002"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto = {
  false, false, 274, descriptor_table_protodef_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto, "modules/common/proto/vehicle_signal.proto", 
  &descriptor_table_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto::offsets,
  file_level_metadata_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto, file_level_enum_descriptors_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto, file_level_service_descriptors_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto_getter() {
  return &descriptor_table_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto(&descriptor_table_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto);
namespace apollo {
namespace common {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* VehicleSignal_TurnSignal_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto);
  return file_level_enum_descriptors_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto[0];
}
bool VehicleSignal_TurnSignal_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
      return true;
    default:
      return false;
  }
}

#if (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)
constexpr VehicleSignal_TurnSignal VehicleSignal::TURN_NONE;
constexpr VehicleSignal_TurnSignal VehicleSignal::TURN_LEFT;
constexpr VehicleSignal_TurnSignal VehicleSignal::TURN_RIGHT;
constexpr VehicleSignal_TurnSignal VehicleSignal::TurnSignal_MIN;
constexpr VehicleSignal_TurnSignal VehicleSignal::TurnSignal_MAX;
constexpr int VehicleSignal::TurnSignal_ARRAYSIZE;
#endif  // (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)

// ===================================================================

class VehicleSignal::_Internal {
 public:
  using HasBits = decltype(std::declval<VehicleSignal>()._has_bits_);
  static void set_has_turn_signal(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_high_beam(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_low_beam(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_horn(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_emergency_light(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
};

VehicleSignal::VehicleSignal(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.common.VehicleSignal)
}
VehicleSignal::VehicleSignal(const VehicleSignal& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&turn_signal_, &from.turn_signal_,
    static_cast<size_t>(reinterpret_cast<char*>(&emergency_light_) -
    reinterpret_cast<char*>(&turn_signal_)) + sizeof(emergency_light_));
  // @@protoc_insertion_point(copy_constructor:apollo.common.VehicleSignal)
}

void VehicleSignal::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&turn_signal_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&emergency_light_) -
    reinterpret_cast<char*>(&turn_signal_)) + sizeof(emergency_light_));
}

VehicleSignal::~VehicleSignal() {
  // @@protoc_insertion_point(destructor:apollo.common.VehicleSignal)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void VehicleSignal::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void VehicleSignal::ArenaDtor(void* object) {
  VehicleSignal* _this = reinterpret_cast< VehicleSignal* >(object);
  (void)_this;
}
void VehicleSignal::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void VehicleSignal::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void VehicleSignal::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.common.VehicleSignal)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    ::memset(&turn_signal_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&emergency_light_) -
        reinterpret_cast<char*>(&turn_signal_)) + sizeof(emergency_light_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* VehicleSignal::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional .apollo.common.VehicleSignal.TurnSignal turn_signal = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::apollo::common::VehicleSignal_TurnSignal_IsValid(val))) {
            _internal_set_turn_signal(static_cast<::apollo::common::VehicleSignal_TurnSignal>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(1, val, mutable_unknown_fields());
          }
        } else
          goto handle_unusual;
        continue;
      // optional bool high_beam = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          _Internal::set_has_high_beam(&has_bits);
          high_beam_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool low_beam = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          _Internal::set_has_low_beam(&has_bits);
          low_beam_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool horn = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 32)) {
          _Internal::set_has_horn(&has_bits);
          horn_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool emergency_light = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 40)) {
          _Internal::set_has_emergency_light(&has_bits);
          emergency_light_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* VehicleSignal::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.common.VehicleSignal)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.VehicleSignal.TurnSignal turn_signal = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      1, this->_internal_turn_signal(), target);
  }

  // optional bool high_beam = 2;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(2, this->_internal_high_beam(), target);
  }

  // optional bool low_beam = 3;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(3, this->_internal_low_beam(), target);
  }

  // optional bool horn = 4;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(4, this->_internal_horn(), target);
  }

  // optional bool emergency_light = 5;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(5, this->_internal_emergency_light(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.common.VehicleSignal)
  return target;
}

size_t VehicleSignal::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.common.VehicleSignal)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    // optional .apollo.common.VehicleSignal.TurnSignal turn_signal = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_turn_signal());
    }

    // optional bool high_beam = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 + 1;
    }

    // optional bool low_beam = 3;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 + 1;
    }

    // optional bool horn = 4;
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 + 1;
    }

    // optional bool emergency_light = 5;
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 + 1;
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData VehicleSignal::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    VehicleSignal::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*VehicleSignal::GetClassData() const { return &_class_data_; }

void VehicleSignal::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<VehicleSignal *>(to)->MergeFrom(
      static_cast<const VehicleSignal &>(from));
}


void VehicleSignal::MergeFrom(const VehicleSignal& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.common.VehicleSignal)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    if (cached_has_bits & 0x00000001u) {
      turn_signal_ = from.turn_signal_;
    }
    if (cached_has_bits & 0x00000002u) {
      high_beam_ = from.high_beam_;
    }
    if (cached_has_bits & 0x00000004u) {
      low_beam_ = from.low_beam_;
    }
    if (cached_has_bits & 0x00000008u) {
      horn_ = from.horn_;
    }
    if (cached_has_bits & 0x00000010u) {
      emergency_light_ = from.emergency_light_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void VehicleSignal::CopyFrom(const VehicleSignal& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.common.VehicleSignal)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool VehicleSignal::IsInitialized() const {
  return true;
}

void VehicleSignal::InternalSwap(VehicleSignal* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(VehicleSignal, emergency_light_)
      + sizeof(VehicleSignal::emergency_light_)
      - PROTOBUF_FIELD_OFFSET(VehicleSignal, turn_signal_)>(
          reinterpret_cast<char*>(&turn_signal_),
          reinterpret_cast<char*>(&other->turn_signal_));
}

::PROTOBUF_NAMESPACE_ID::Metadata VehicleSignal::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto_getter, &descriptor_table_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto_once,
      file_level_metadata_modules_2fcommon_2fproto_2fvehicle_5fsignal_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace common
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::common::VehicleSignal* Arena::CreateMaybeMessage< ::apollo::common::VehicleSignal >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::common::VehicleSignal >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
