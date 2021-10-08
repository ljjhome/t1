// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/v2x/proto/v2x_rsi.proto

#include "modules/v2x/proto/v2x_rsi.pb.h"

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
namespace v2x {
constexpr RsiPoint::RsiPoint(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : x_(0)
  , y_(0){}
struct RsiPointDefaultTypeInternal {
  constexpr RsiPointDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~RsiPointDefaultTypeInternal() {}
  union {
    RsiPoint _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT RsiPointDefaultTypeInternal _RsiPoint_default_instance_;
constexpr RsiMsg::RsiMsg(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : points_()
  , description_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , header_(nullptr)
  , speed_(0)
  , low_speed_(0)
  , high_speed_(0)
  , radius_(0)
  , rsi_type_(0)
  , priority_(0){}
struct RsiMsgDefaultTypeInternal {
  constexpr RsiMsgDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~RsiMsgDefaultTypeInternal() {}
  union {
    RsiMsg _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT RsiMsgDefaultTypeInternal _RsiMsg_default_instance_;
}  // namespace v2x
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::RsiPoint, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::RsiPoint, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::RsiPoint, x_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::RsiPoint, y_),
  0,
  1,
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::RsiMsg, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::RsiMsg, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::RsiMsg, header_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::RsiMsg, points_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::RsiMsg, speed_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::RsiMsg, low_speed_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::RsiMsg, high_speed_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::RsiMsg, description_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::RsiMsg, rsi_type_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::RsiMsg, radius_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::RsiMsg, priority_),
  1,
  ~0u,
  2,
  3,
  4,
  0,
  6,
  5,
  7,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, -1, sizeof(::apollo::v2x::RsiPoint)},
  { 10, 25, -1, sizeof(::apollo::v2x::RsiMsg)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::v2x::_RsiPoint_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::v2x::_RsiMsg_default_instance_),
};

const char descriptor_table_protodef_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\037modules/v2x/proto/v2x_rsi.proto\022\napoll"
  "o.v2x\032!modules/common/proto/header.proto"
  "\" \n\010RsiPoint\022\t\n\001x\030\001 \001(\001\022\t\n\001y\030\002 \001(\001\"\324\001\n\006R"
  "siMsg\022%\n\006header\030\001 \001(\0132\025.apollo.common.He"
  "ader\022$\n\006points\030\002 \003(\0132\024.apollo.v2x.RsiPoi"
  "nt\022\r\n\005speed\030\003 \001(\001\022\021\n\tlow_speed\030\004 \001(\001\022\022\n\n"
  "high_speed\030\005 \001(\001\022\023\n\013description\030\006 \001(\t\022\020\n"
  "\010rsi_type\030\007 \001(\005\022\016\n\006radius\030\010 \001(\001\022\020\n\010prior"
  "ity\030\t \001(\005"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto_deps[1] = {
  &::descriptor_table_modules_2fcommon_2fproto_2fheader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto = {
  false, false, 329, descriptor_table_protodef_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto, "modules/v2x/proto/v2x_rsi.proto", 
  &descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto_once, descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto_deps, 1, 2,
  schemas, file_default_instances, TableStruct_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto::offsets,
  file_level_metadata_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto, file_level_enum_descriptors_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto, file_level_service_descriptors_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto_getter() {
  return &descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto(&descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto);
namespace apollo {
namespace v2x {

// ===================================================================

class RsiPoint::_Internal {
 public:
  using HasBits = decltype(std::declval<RsiPoint>()._has_bits_);
  static void set_has_x(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_y(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

RsiPoint::RsiPoint(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.v2x.RsiPoint)
}
RsiPoint::RsiPoint(const RsiPoint& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&x_, &from.x_,
    static_cast<size_t>(reinterpret_cast<char*>(&y_) -
    reinterpret_cast<char*>(&x_)) + sizeof(y_));
  // @@protoc_insertion_point(copy_constructor:apollo.v2x.RsiPoint)
}

void RsiPoint::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&x_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&y_) -
    reinterpret_cast<char*>(&x_)) + sizeof(y_));
}

RsiPoint::~RsiPoint() {
  // @@protoc_insertion_point(destructor:apollo.v2x.RsiPoint)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void RsiPoint::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void RsiPoint::ArenaDtor(void* object) {
  RsiPoint* _this = reinterpret_cast< RsiPoint* >(object);
  (void)_this;
}
void RsiPoint::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void RsiPoint::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void RsiPoint::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.v2x.RsiPoint)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    ::memset(&x_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&y_) -
        reinterpret_cast<char*>(&x_)) + sizeof(y_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* RsiPoint::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional double x = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          _Internal::set_has_x(&has_bits);
          x_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional double y = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 17)) {
          _Internal::set_has_y(&has_bits);
          y_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* RsiPoint::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.v2x.RsiPoint)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double x = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_x(), target);
  }

  // optional double y = 2;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_y(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.v2x.RsiPoint)
  return target;
}

size_t RsiPoint::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.v2x.RsiPoint)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    // optional double x = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 + 8;
    }

    // optional double y = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 + 8;
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData RsiPoint::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    RsiPoint::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*RsiPoint::GetClassData() const { return &_class_data_; }

void RsiPoint::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<RsiPoint *>(to)->MergeFrom(
      static_cast<const RsiPoint &>(from));
}


void RsiPoint::MergeFrom(const RsiPoint& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.v2x.RsiPoint)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      x_ = from.x_;
    }
    if (cached_has_bits & 0x00000002u) {
      y_ = from.y_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void RsiPoint::CopyFrom(const RsiPoint& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.v2x.RsiPoint)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RsiPoint::IsInitialized() const {
  return true;
}

void RsiPoint::InternalSwap(RsiPoint* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(RsiPoint, y_)
      + sizeof(RsiPoint::y_)
      - PROTOBUF_FIELD_OFFSET(RsiPoint, x_)>(
          reinterpret_cast<char*>(&x_),
          reinterpret_cast<char*>(&other->x_));
}

::PROTOBUF_NAMESPACE_ID::Metadata RsiPoint::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto_getter, &descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto_once,
      file_level_metadata_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto[0]);
}

// ===================================================================

class RsiMsg::_Internal {
 public:
  using HasBits = decltype(std::declval<RsiMsg>()._has_bits_);
  static const ::apollo::common::Header& header(const RsiMsg* msg);
  static void set_has_header(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_speed(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_low_speed(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_high_speed(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static void set_has_description(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_rsi_type(HasBits* has_bits) {
    (*has_bits)[0] |= 64u;
  }
  static void set_has_radius(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static void set_has_priority(HasBits* has_bits) {
    (*has_bits)[0] |= 128u;
  }
};

const ::apollo::common::Header&
RsiMsg::_Internal::header(const RsiMsg* msg) {
  return *msg->header_;
}
void RsiMsg::clear_header() {
  if (header_ != nullptr) header_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
RsiMsg::RsiMsg(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  points_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.v2x.RsiMsg)
}
RsiMsg::RsiMsg(const RsiMsg& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      points_(from.points_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  description_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_description()) {
    description_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_description(), 
      GetArenaForAllocation());
  }
  if (from._internal_has_header()) {
    header_ = new ::apollo::common::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  ::memcpy(&speed_, &from.speed_,
    static_cast<size_t>(reinterpret_cast<char*>(&priority_) -
    reinterpret_cast<char*>(&speed_)) + sizeof(priority_));
  // @@protoc_insertion_point(copy_constructor:apollo.v2x.RsiMsg)
}

void RsiMsg::SharedCtor() {
description_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&priority_) -
    reinterpret_cast<char*>(&header_)) + sizeof(priority_));
}

RsiMsg::~RsiMsg() {
  // @@protoc_insertion_point(destructor:apollo.v2x.RsiMsg)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void RsiMsg::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  description_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (this != internal_default_instance()) delete header_;
}

void RsiMsg::ArenaDtor(void* object) {
  RsiMsg* _this = reinterpret_cast< RsiMsg* >(object);
  (void)_this;
}
void RsiMsg::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void RsiMsg::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void RsiMsg::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.v2x.RsiMsg)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  points_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      description_.ClearNonDefaultToEmpty();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(header_ != nullptr);
      header_->Clear();
    }
  }
  if (cached_has_bits & 0x000000fcu) {
    ::memset(&speed_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&priority_) -
        reinterpret_cast<char*>(&speed_)) + sizeof(priority_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* RsiMsg::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional .apollo.common.Header header = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_header(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated .apollo.v2x.RsiPoint points = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_points(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<18>(ptr));
        } else
          goto handle_unusual;
        continue;
      // optional double speed = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 25)) {
          _Internal::set_has_speed(&has_bits);
          speed_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional double low_speed = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 33)) {
          _Internal::set_has_low_speed(&has_bits);
          low_speed_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional double high_speed = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 41)) {
          _Internal::set_has_high_speed(&has_bits);
          high_speed_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional string description = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 50)) {
          auto str = _internal_mutable_description();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.v2x.RsiMsg.description");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional int32 rsi_type = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 56)) {
          _Internal::set_has_rsi_type(&has_bits);
          rsi_type_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional double radius = 8;
      case 8:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 65)) {
          _Internal::set_has_radius(&has_bits);
          radius_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional int32 priority = 9;
      case 9:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 72)) {
          _Internal::set_has_priority(&has_bits);
          priority_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* RsiMsg::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.v2x.RsiMsg)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.Header header = 1;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // repeated .apollo.v2x.RsiPoint points = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_points_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, this->_internal_points(i), target, stream);
  }

  // optional double speed = 3;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(3, this->_internal_speed(), target);
  }

  // optional double low_speed = 4;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(4, this->_internal_low_speed(), target);
  }

  // optional double high_speed = 5;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(5, this->_internal_high_speed(), target);
  }

  // optional string description = 6;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_description().data(), static_cast<int>(this->_internal_description().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.v2x.RsiMsg.description");
    target = stream->WriteStringMaybeAliased(
        6, this->_internal_description(), target);
  }

  // optional int32 rsi_type = 7;
  if (cached_has_bits & 0x00000040u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(7, this->_internal_rsi_type(), target);
  }

  // optional double radius = 8;
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(8, this->_internal_radius(), target);
  }

  // optional int32 priority = 9;
  if (cached_has_bits & 0x00000080u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(9, this->_internal_priority(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.v2x.RsiMsg)
  return target;
}

size_t RsiMsg::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.v2x.RsiMsg)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .apollo.v2x.RsiPoint points = 2;
  total_size += 1UL * this->_internal_points_size();
  for (const auto& msg : this->points_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    // optional string description = 6;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_description());
    }

    // optional .apollo.common.Header header = 1;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *header_);
    }

    // optional double speed = 3;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 + 8;
    }

    // optional double low_speed = 4;
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 + 8;
    }

    // optional double high_speed = 5;
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 + 8;
    }

    // optional double radius = 8;
    if (cached_has_bits & 0x00000020u) {
      total_size += 1 + 8;
    }

    // optional int32 rsi_type = 7;
    if (cached_has_bits & 0x00000040u) {
      total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32SizePlusOne(this->_internal_rsi_type());
    }

    // optional int32 priority = 9;
    if (cached_has_bits & 0x00000080u) {
      total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32SizePlusOne(this->_internal_priority());
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData RsiMsg::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    RsiMsg::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*RsiMsg::GetClassData() const { return &_class_data_; }

void RsiMsg::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<RsiMsg *>(to)->MergeFrom(
      static_cast<const RsiMsg &>(from));
}


void RsiMsg::MergeFrom(const RsiMsg& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.v2x.RsiMsg)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  points_.MergeFrom(from.points_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_set_description(from._internal_description());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_header()->::apollo::common::Header::MergeFrom(from._internal_header());
    }
    if (cached_has_bits & 0x00000004u) {
      speed_ = from.speed_;
    }
    if (cached_has_bits & 0x00000008u) {
      low_speed_ = from.low_speed_;
    }
    if (cached_has_bits & 0x00000010u) {
      high_speed_ = from.high_speed_;
    }
    if (cached_has_bits & 0x00000020u) {
      radius_ = from.radius_;
    }
    if (cached_has_bits & 0x00000040u) {
      rsi_type_ = from.rsi_type_;
    }
    if (cached_has_bits & 0x00000080u) {
      priority_ = from.priority_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void RsiMsg::CopyFrom(const RsiMsg& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.v2x.RsiMsg)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RsiMsg::IsInitialized() const {
  return true;
}

void RsiMsg::InternalSwap(RsiMsg* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  points_.InternalSwap(&other->points_);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &description_, lhs_arena,
      &other->description_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(RsiMsg, priority_)
      + sizeof(RsiMsg::priority_)
      - PROTOBUF_FIELD_OFFSET(RsiMsg, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata RsiMsg::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto_getter, &descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto_once,
      file_level_metadata_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto[1]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace v2x
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::v2x::RsiPoint* Arena::CreateMaybeMessage< ::apollo::v2x::RsiPoint >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::v2x::RsiPoint >(arena);
}
template<> PROTOBUF_NOINLINE ::apollo::v2x::RsiMsg* Arena::CreateMaybeMessage< ::apollo::v2x::RsiMsg >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::v2x::RsiMsg >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
