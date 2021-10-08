// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/drivers/radar/ultrasonic_radar/proto/ultrasonic_radar_conf.proto

#include "modules/drivers/radar/ultrasonic_radar/proto/ultrasonic_radar_conf.pb.h"

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
namespace drivers {
namespace ultrasonic_radar {
constexpr CanConf::CanConf(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : can_card_parameter_(nullptr)
  , enable_debug_mode_(false)
  , enable_receiver_log_(false)
  , enable_sender_log_(false){}
struct CanConfDefaultTypeInternal {
  constexpr CanConfDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~CanConfDefaultTypeInternal() {}
  union {
    CanConf _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT CanConfDefaultTypeInternal _CanConf_default_instance_;
constexpr UltrasonicRadarConf::UltrasonicRadarConf(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : can_conf_(nullptr)
  , entrance_num_(0){}
struct UltrasonicRadarConfDefaultTypeInternal {
  constexpr UltrasonicRadarConfDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~UltrasonicRadarConfDefaultTypeInternal() {}
  union {
    UltrasonicRadarConf _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT UltrasonicRadarConfDefaultTypeInternal _UltrasonicRadarConf_default_instance_;
}  // namespace ultrasonic_radar
}  // namespace drivers
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::ultrasonic_radar::CanConf, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::ultrasonic_radar::CanConf, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::ultrasonic_radar::CanConf, can_card_parameter_),
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::ultrasonic_radar::CanConf, enable_debug_mode_),
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::ultrasonic_radar::CanConf, enable_receiver_log_),
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::ultrasonic_radar::CanConf, enable_sender_log_),
  0,
  1,
  2,
  3,
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::ultrasonic_radar::UltrasonicRadarConf, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::ultrasonic_radar::UltrasonicRadarConf, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::ultrasonic_radar::UltrasonicRadarConf, can_conf_),
  PROTOBUF_FIELD_OFFSET(::apollo::drivers::ultrasonic_radar::UltrasonicRadarConf, entrance_num_),
  0,
  1,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 10, -1, sizeof(::apollo::drivers::ultrasonic_radar::CanConf)},
  { 14, 22, -1, sizeof(::apollo::drivers::ultrasonic_radar::UltrasonicRadarConf)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::drivers::ultrasonic_radar::_CanConf_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::drivers::ultrasonic_radar::_UltrasonicRadarConf_default_instance_),
};

const char descriptor_table_protodef_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\nHmodules/drivers/radar/ultrasonic_radar"
  "/proto/ultrasonic_radar_conf.proto\022\037apol"
  "lo.drivers.ultrasonic_radar\0325modules/dri"
  "vers/canbus/proto/can_card_parameter.pro"
  "to\"\266\001\n\007CanConf\022C\n\022can_card_parameter\030\001 \001"
  "(\0132\'.apollo.drivers.canbus.CANCardParame"
  "ter\022 \n\021enable_debug_mode\030\002 \001(\010:\005false\022\"\n"
  "\023enable_receiver_log\030\003 \001(\010:\005false\022 \n\021ena"
  "ble_sender_log\030\004 \001(\010:\005false\"g\n\023Ultrasoni"
  "cRadarConf\022:\n\010can_conf\030\001 \001(\0132(.apollo.dr"
  "ivers.ultrasonic_radar.CanConf\022\024\n\014entran"
  "ce_num\030\002 \001(\005"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto_deps[1] = {
  &::descriptor_table_modules_2fdrivers_2fcanbus_2fproto_2fcan_5fcard_5fparameter_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto = {
  false, false, 452, descriptor_table_protodef_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto, "modules/drivers/radar/ultrasonic_radar/proto/ultrasonic_radar_conf.proto", 
  &descriptor_table_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto_once, descriptor_table_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto_deps, 1, 2,
  schemas, file_default_instances, TableStruct_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto::offsets,
  file_level_metadata_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto, file_level_enum_descriptors_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto, file_level_service_descriptors_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto_getter() {
  return &descriptor_table_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto(&descriptor_table_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto);
namespace apollo {
namespace drivers {
namespace ultrasonic_radar {

// ===================================================================

class CanConf::_Internal {
 public:
  using HasBits = decltype(std::declval<CanConf>()._has_bits_);
  static const ::apollo::drivers::canbus::CANCardParameter& can_card_parameter(const CanConf* msg);
  static void set_has_can_card_parameter(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_enable_debug_mode(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_enable_receiver_log(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_enable_sender_log(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
};

const ::apollo::drivers::canbus::CANCardParameter&
CanConf::_Internal::can_card_parameter(const CanConf* msg) {
  return *msg->can_card_parameter_;
}
void CanConf::clear_can_card_parameter() {
  if (can_card_parameter_ != nullptr) can_card_parameter_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
CanConf::CanConf(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.drivers.ultrasonic_radar.CanConf)
}
CanConf::CanConf(const CanConf& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_can_card_parameter()) {
    can_card_parameter_ = new ::apollo::drivers::canbus::CANCardParameter(*from.can_card_parameter_);
  } else {
    can_card_parameter_ = nullptr;
  }
  ::memcpy(&enable_debug_mode_, &from.enable_debug_mode_,
    static_cast<size_t>(reinterpret_cast<char*>(&enable_sender_log_) -
    reinterpret_cast<char*>(&enable_debug_mode_)) + sizeof(enable_sender_log_));
  // @@protoc_insertion_point(copy_constructor:apollo.drivers.ultrasonic_radar.CanConf)
}

void CanConf::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&can_card_parameter_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&enable_sender_log_) -
    reinterpret_cast<char*>(&can_card_parameter_)) + sizeof(enable_sender_log_));
}

CanConf::~CanConf() {
  // @@protoc_insertion_point(destructor:apollo.drivers.ultrasonic_radar.CanConf)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void CanConf::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete can_card_parameter_;
}

void CanConf::ArenaDtor(void* object) {
  CanConf* _this = reinterpret_cast< CanConf* >(object);
  (void)_this;
}
void CanConf::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void CanConf::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void CanConf::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.drivers.ultrasonic_radar.CanConf)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(can_card_parameter_ != nullptr);
    can_card_parameter_->Clear();
  }
  ::memset(&enable_debug_mode_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&enable_sender_log_) -
      reinterpret_cast<char*>(&enable_debug_mode_)) + sizeof(enable_sender_log_));
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* CanConf::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional .apollo.drivers.canbus.CANCardParameter can_card_parameter = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_can_card_parameter(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool enable_debug_mode = 2 [default = false];
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          _Internal::set_has_enable_debug_mode(&has_bits);
          enable_debug_mode_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool enable_receiver_log = 3 [default = false];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          _Internal::set_has_enable_receiver_log(&has_bits);
          enable_receiver_log_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool enable_sender_log = 4 [default = false];
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 32)) {
          _Internal::set_has_enable_sender_log(&has_bits);
          enable_sender_log_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* CanConf::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.drivers.ultrasonic_radar.CanConf)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.drivers.canbus.CANCardParameter can_card_parameter = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::can_card_parameter(this), target, stream);
  }

  // optional bool enable_debug_mode = 2 [default = false];
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(2, this->_internal_enable_debug_mode(), target);
  }

  // optional bool enable_receiver_log = 3 [default = false];
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(3, this->_internal_enable_receiver_log(), target);
  }

  // optional bool enable_sender_log = 4 [default = false];
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(4, this->_internal_enable_sender_log(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.drivers.ultrasonic_radar.CanConf)
  return target;
}

size_t CanConf::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.drivers.ultrasonic_radar.CanConf)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    // optional .apollo.drivers.canbus.CANCardParameter can_card_parameter = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *can_card_parameter_);
    }

    // optional bool enable_debug_mode = 2 [default = false];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 + 1;
    }

    // optional bool enable_receiver_log = 3 [default = false];
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 + 1;
    }

    // optional bool enable_sender_log = 4 [default = false];
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 + 1;
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData CanConf::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    CanConf::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*CanConf::GetClassData() const { return &_class_data_; }

void CanConf::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<CanConf *>(to)->MergeFrom(
      static_cast<const CanConf &>(from));
}


void CanConf::MergeFrom(const CanConf& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.drivers.ultrasonic_radar.CanConf)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_can_card_parameter()->::apollo::drivers::canbus::CANCardParameter::MergeFrom(from._internal_can_card_parameter());
    }
    if (cached_has_bits & 0x00000002u) {
      enable_debug_mode_ = from.enable_debug_mode_;
    }
    if (cached_has_bits & 0x00000004u) {
      enable_receiver_log_ = from.enable_receiver_log_;
    }
    if (cached_has_bits & 0x00000008u) {
      enable_sender_log_ = from.enable_sender_log_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void CanConf::CopyFrom(const CanConf& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.drivers.ultrasonic_radar.CanConf)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CanConf::IsInitialized() const {
  return true;
}

void CanConf::InternalSwap(CanConf* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(CanConf, enable_sender_log_)
      + sizeof(CanConf::enable_sender_log_)
      - PROTOBUF_FIELD_OFFSET(CanConf, can_card_parameter_)>(
          reinterpret_cast<char*>(&can_card_parameter_),
          reinterpret_cast<char*>(&other->can_card_parameter_));
}

::PROTOBUF_NAMESPACE_ID::Metadata CanConf::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto_getter, &descriptor_table_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto_once,
      file_level_metadata_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto[0]);
}

// ===================================================================

class UltrasonicRadarConf::_Internal {
 public:
  using HasBits = decltype(std::declval<UltrasonicRadarConf>()._has_bits_);
  static const ::apollo::drivers::ultrasonic_radar::CanConf& can_conf(const UltrasonicRadarConf* msg);
  static void set_has_can_conf(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_entrance_num(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

const ::apollo::drivers::ultrasonic_radar::CanConf&
UltrasonicRadarConf::_Internal::can_conf(const UltrasonicRadarConf* msg) {
  return *msg->can_conf_;
}
UltrasonicRadarConf::UltrasonicRadarConf(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.drivers.ultrasonic_radar.UltrasonicRadarConf)
}
UltrasonicRadarConf::UltrasonicRadarConf(const UltrasonicRadarConf& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_can_conf()) {
    can_conf_ = new ::apollo::drivers::ultrasonic_radar::CanConf(*from.can_conf_);
  } else {
    can_conf_ = nullptr;
  }
  entrance_num_ = from.entrance_num_;
  // @@protoc_insertion_point(copy_constructor:apollo.drivers.ultrasonic_radar.UltrasonicRadarConf)
}

void UltrasonicRadarConf::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&can_conf_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&entrance_num_) -
    reinterpret_cast<char*>(&can_conf_)) + sizeof(entrance_num_));
}

UltrasonicRadarConf::~UltrasonicRadarConf() {
  // @@protoc_insertion_point(destructor:apollo.drivers.ultrasonic_radar.UltrasonicRadarConf)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void UltrasonicRadarConf::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete can_conf_;
}

void UltrasonicRadarConf::ArenaDtor(void* object) {
  UltrasonicRadarConf* _this = reinterpret_cast< UltrasonicRadarConf* >(object);
  (void)_this;
}
void UltrasonicRadarConf::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void UltrasonicRadarConf::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void UltrasonicRadarConf::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.drivers.ultrasonic_radar.UltrasonicRadarConf)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(can_conf_ != nullptr);
    can_conf_->Clear();
  }
  entrance_num_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* UltrasonicRadarConf::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional .apollo.drivers.ultrasonic_radar.CanConf can_conf = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_can_conf(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional int32 entrance_num = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          _Internal::set_has_entrance_num(&has_bits);
          entrance_num_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* UltrasonicRadarConf::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.drivers.ultrasonic_radar.UltrasonicRadarConf)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.drivers.ultrasonic_radar.CanConf can_conf = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::can_conf(this), target, stream);
  }

  // optional int32 entrance_num = 2;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(2, this->_internal_entrance_num(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.drivers.ultrasonic_radar.UltrasonicRadarConf)
  return target;
}

size_t UltrasonicRadarConf::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.drivers.ultrasonic_radar.UltrasonicRadarConf)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    // optional .apollo.drivers.ultrasonic_radar.CanConf can_conf = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *can_conf_);
    }

    // optional int32 entrance_num = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32SizePlusOne(this->_internal_entrance_num());
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData UltrasonicRadarConf::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    UltrasonicRadarConf::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*UltrasonicRadarConf::GetClassData() const { return &_class_data_; }

void UltrasonicRadarConf::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<UltrasonicRadarConf *>(to)->MergeFrom(
      static_cast<const UltrasonicRadarConf &>(from));
}


void UltrasonicRadarConf::MergeFrom(const UltrasonicRadarConf& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.drivers.ultrasonic_radar.UltrasonicRadarConf)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_can_conf()->::apollo::drivers::ultrasonic_radar::CanConf::MergeFrom(from._internal_can_conf());
    }
    if (cached_has_bits & 0x00000002u) {
      entrance_num_ = from.entrance_num_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void UltrasonicRadarConf::CopyFrom(const UltrasonicRadarConf& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.drivers.ultrasonic_radar.UltrasonicRadarConf)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool UltrasonicRadarConf::IsInitialized() const {
  return true;
}

void UltrasonicRadarConf::InternalSwap(UltrasonicRadarConf* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(UltrasonicRadarConf, entrance_num_)
      + sizeof(UltrasonicRadarConf::entrance_num_)
      - PROTOBUF_FIELD_OFFSET(UltrasonicRadarConf, can_conf_)>(
          reinterpret_cast<char*>(&can_conf_),
          reinterpret_cast<char*>(&other->can_conf_));
}

::PROTOBUF_NAMESPACE_ID::Metadata UltrasonicRadarConf::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto_getter, &descriptor_table_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto_once,
      file_level_metadata_modules_2fdrivers_2fradar_2fultrasonic_5fradar_2fproto_2fultrasonic_5fradar_5fconf_2eproto[1]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace ultrasonic_radar
}  // namespace drivers
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::drivers::ultrasonic_radar::CanConf* Arena::CreateMaybeMessage< ::apollo::drivers::ultrasonic_radar::CanConf >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::drivers::ultrasonic_radar::CanConf >(arena);
}
template<> PROTOBUF_NOINLINE ::apollo::drivers::ultrasonic_radar::UltrasonicRadarConf* Arena::CreateMaybeMessage< ::apollo::drivers::ultrasonic_radar::UltrasonicRadarConf >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::drivers::ultrasonic_radar::UltrasonicRadarConf >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>