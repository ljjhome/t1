// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/common/proto/error_code.proto

#include "modules/common/proto/error_code.pb.h"

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
constexpr StatusPb::StatusPb(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : msg_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , error_code_(0)
{}
struct StatusPbDefaultTypeInternal {
  constexpr StatusPbDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~StatusPbDefaultTypeInternal() {}
  union {
    StatusPb _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT StatusPbDefaultTypeInternal _StatusPb_default_instance_;
}  // namespace common
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fcommon_2fproto_2ferror_5fcode_2eproto[1];
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_modules_2fcommon_2fproto_2ferror_5fcode_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fcommon_2fproto_2ferror_5fcode_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fcommon_2fproto_2ferror_5fcode_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::common::StatusPb, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::common::StatusPb, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::common::StatusPb, error_code_),
  PROTOBUF_FIELD_OFFSET(::apollo::common::StatusPb, msg_),
  1,
  0,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, -1, sizeof(::apollo::common::StatusPb)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::common::_StatusPb_default_instance_),
};

const char descriptor_table_protodef_modules_2fcommon_2fproto_2ferror_5fcode_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n%modules/common/proto/error_code.proto\022"
  "\rapollo.common\"I\n\010StatusPb\0220\n\nerror_code"
  "\030\001 \001(\0162\030.apollo.common.ErrorCode:\002OK\022\013\n\003"
  "msg\030\002 \001(\t*\231\010\n\tErrorCode\022\006\n\002OK\020\000\022\022\n\rCONTR"
  "OL_ERROR\020\350\007\022\027\n\022CONTROL_INIT_ERROR\020\351\007\022\032\n\025"
  "CONTROL_COMPUTE_ERROR\020\352\007\022\030\n\023CONTROL_ESTO"
  "P_ERROR\020\353\007\022\032\n\025PERFECT_CONTROL_ERROR\020\354\007\022\021"
  "\n\014CANBUS_ERROR\020\320\017\022\032\n\025CAN_CLIENT_ERROR_BA"
  "SE\020\264\020\022(\n#CAN_CLIENT_ERROR_OPEN_DEVICE_FA"
  "ILED\020\265\020\022\037\n\032CAN_CLIENT_ERROR_FRAME_NUM\020\266\020"
  "\022!\n\034CAN_CLIENT_ERROR_SEND_FAILED\020\267\020\022!\n\034C"
  "AN_CLIENT_ERROR_RECV_FAILED\020\270\020\022\027\n\022LOCALI"
  "ZATION_ERROR\020\270\027\022\033\n\026LOCALIZATION_ERROR_MS"
  "G\020\234\030\022\035\n\030LOCALIZATION_ERROR_LIDAR\020\200\031\022\035\n\030L"
  "OCALIZATION_ERROR_INTEG\020\344\031\022\034\n\027LOCALIZATI"
  "ON_ERROR_GNSS\020\310\032\022\025\n\020PERCEPTION_ERROR\020\240\037\022"
  "\030\n\023PERCEPTION_ERROR_TF\020\241\037\022\035\n\030PERCEPTION_"
  "ERROR_PROCESS\020\242\037\022\025\n\020PERCEPTION_FATAL\020\243\037\022"
  "\032\n\025PERCEPTION_ERROR_NONE\020\244\037\022\035\n\030PERCEPTIO"
  "N_ERROR_UNKNOWN\020\245\037\022\025\n\020PREDICTION_ERROR\020\210"
  "\'\022\023\n\016PLANNING_ERROR\020\360.\022\035\n\030PLANNING_ERROR"
  "_NOT_READY\020\361.\022\025\n\020HDMAP_DATA_ERROR\020\3306\022\022\n\r"
  "ROUTING_ERROR\020\300>\022\032\n\025ROUTING_ERROR_REQUES"
  "T\020\301>\022\033\n\026ROUTING_ERROR_RESPONSE\020\302>\022\034\n\027ROU"
  "TING_ERROR_NOT_READY\020\303>\022\021\n\014END_OF_INPUT\020"
  "\250F\022\025\n\020HTTP_LOGIC_ERROR\020\220N\022\027\n\022HTTP_RUNTIM"
  "E_ERROR\020\221N\022\027\n\022RELATIVE_MAP_ERROR\020\370U\022\033\n\026R"
  "ELATIVE_MAP_NOT_READY\020\371U\022\026\n\021DRIVER_ERROR"
  "_GNSS\020\340]\022\032\n\025DRIVER_ERROR_VELODYNE\020\310e\022\027\n\022"
  "STORYTELLING_ERROR\020\260m"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto = {
  false, false, 1181, descriptor_table_protodef_modules_2fcommon_2fproto_2ferror_5fcode_2eproto, "modules/common/proto/error_code.proto", 
  &descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_modules_2fcommon_2fproto_2ferror_5fcode_2eproto::offsets,
  file_level_metadata_modules_2fcommon_2fproto_2ferror_5fcode_2eproto, file_level_enum_descriptors_modules_2fcommon_2fproto_2ferror_5fcode_2eproto, file_level_service_descriptors_modules_2fcommon_2fproto_2ferror_5fcode_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto_getter() {
  return &descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fcommon_2fproto_2ferror_5fcode_2eproto(&descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto);
namespace apollo {
namespace common {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* ErrorCode_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto);
  return file_level_enum_descriptors_modules_2fcommon_2fproto_2ferror_5fcode_2eproto[0];
}
bool ErrorCode_IsValid(int value) {
  switch (value) {
    case 0:
    case 1000:
    case 1001:
    case 1002:
    case 1003:
    case 1004:
    case 2000:
    case 2100:
    case 2101:
    case 2102:
    case 2103:
    case 2104:
    case 3000:
    case 3100:
    case 3200:
    case 3300:
    case 3400:
    case 4000:
    case 4001:
    case 4002:
    case 4003:
    case 4004:
    case 4005:
    case 5000:
    case 6000:
    case 6001:
    case 7000:
    case 8000:
    case 8001:
    case 8002:
    case 8003:
    case 9000:
    case 10000:
    case 10001:
    case 11000:
    case 11001:
    case 12000:
    case 13000:
    case 14000:
      return true;
    default:
      return false;
  }
}


// ===================================================================

class StatusPb::_Internal {
 public:
  using HasBits = decltype(std::declval<StatusPb>()._has_bits_);
  static void set_has_error_code(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_msg(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

StatusPb::StatusPb(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.common.StatusPb)
}
StatusPb::StatusPb(const StatusPb& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  msg_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_msg()) {
    msg_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_msg(), 
      GetArenaForAllocation());
  }
  error_code_ = from.error_code_;
  // @@protoc_insertion_point(copy_constructor:apollo.common.StatusPb)
}

void StatusPb::SharedCtor() {
msg_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
error_code_ = 0;
}

StatusPb::~StatusPb() {
  // @@protoc_insertion_point(destructor:apollo.common.StatusPb)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void StatusPb::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  msg_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void StatusPb::ArenaDtor(void* object) {
  StatusPb* _this = reinterpret_cast< StatusPb* >(object);
  (void)_this;
}
void StatusPb::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void StatusPb::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void StatusPb::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.common.StatusPb)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    msg_.ClearNonDefaultToEmpty();
  }
  error_code_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* StatusPb::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional .apollo.common.ErrorCode error_code = 1 [default = OK];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::apollo::common::ErrorCode_IsValid(val))) {
            _internal_set_error_code(static_cast<::apollo::common::ErrorCode>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(1, val, mutable_unknown_fields());
          }
        } else
          goto handle_unusual;
        continue;
      // optional string msg = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          auto str = _internal_mutable_msg();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.common.StatusPb.msg");
          #endif  // !NDEBUG
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

::PROTOBUF_NAMESPACE_ID::uint8* StatusPb::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.common.StatusPb)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.ErrorCode error_code = 1 [default = OK];
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      1, this->_internal_error_code(), target);
  }

  // optional string msg = 2;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_msg().data(), static_cast<int>(this->_internal_msg().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.common.StatusPb.msg");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_msg(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.common.StatusPb)
  return target;
}

size_t StatusPb::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.common.StatusPb)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    // optional string msg = 2;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_msg());
    }

    // optional .apollo.common.ErrorCode error_code = 1 [default = OK];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_error_code());
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData StatusPb::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    StatusPb::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*StatusPb::GetClassData() const { return &_class_data_; }

void StatusPb::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<StatusPb *>(to)->MergeFrom(
      static_cast<const StatusPb &>(from));
}


void StatusPb::MergeFrom(const StatusPb& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.common.StatusPb)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_set_msg(from._internal_msg());
    }
    if (cached_has_bits & 0x00000002u) {
      error_code_ = from.error_code_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void StatusPb::CopyFrom(const StatusPb& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.common.StatusPb)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool StatusPb::IsInitialized() const {
  return true;
}

void StatusPb::InternalSwap(StatusPb* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &msg_, lhs_arena,
      &other->msg_, rhs_arena
  );
  swap(error_code_, other->error_code_);
}

::PROTOBUF_NAMESPACE_ID::Metadata StatusPb::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto_getter, &descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto_once,
      file_level_metadata_modules_2fcommon_2fproto_2ferror_5fcode_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace common
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::common::StatusPb* Arena::CreateMaybeMessage< ::apollo::common::StatusPb >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::common::StatusPb >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
