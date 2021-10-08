// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/routing/proto/routing_config.proto

#include "modules/routing/proto/routing_config.pb.h"

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
namespace routing {
constexpr TopicConfig::TopicConfig(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : routing_response_topic_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , routing_response_history_topic_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string){}
struct TopicConfigDefaultTypeInternal {
  constexpr TopicConfigDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~TopicConfigDefaultTypeInternal() {}
  union {
    TopicConfig _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT TopicConfigDefaultTypeInternal _TopicConfig_default_instance_;
constexpr RoutingConfig::RoutingConfig(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : topic_config_(nullptr)
  , base_speed_(0)
  , left_turn_penalty_(0)
  , right_turn_penalty_(0)
  , uturn_penalty_(0)
  , change_penalty_(0)
  , base_changing_length_(0){}
struct RoutingConfigDefaultTypeInternal {
  constexpr RoutingConfigDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~RoutingConfigDefaultTypeInternal() {}
  union {
    RoutingConfig _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT RoutingConfigDefaultTypeInternal _RoutingConfig_default_instance_;
}  // namespace routing
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2frouting_2fproto_2frouting_5fconfig_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2frouting_2fproto_2frouting_5fconfig_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2frouting_2fproto_2frouting_5fconfig_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2frouting_2fproto_2frouting_5fconfig_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::routing::TopicConfig, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::routing::TopicConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::routing::TopicConfig, routing_response_topic_),
  PROTOBUF_FIELD_OFFSET(::apollo::routing::TopicConfig, routing_response_history_topic_),
  0,
  1,
  PROTOBUF_FIELD_OFFSET(::apollo::routing::RoutingConfig, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::routing::RoutingConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::routing::RoutingConfig, base_speed_),
  PROTOBUF_FIELD_OFFSET(::apollo::routing::RoutingConfig, left_turn_penalty_),
  PROTOBUF_FIELD_OFFSET(::apollo::routing::RoutingConfig, right_turn_penalty_),
  PROTOBUF_FIELD_OFFSET(::apollo::routing::RoutingConfig, uturn_penalty_),
  PROTOBUF_FIELD_OFFSET(::apollo::routing::RoutingConfig, change_penalty_),
  PROTOBUF_FIELD_OFFSET(::apollo::routing::RoutingConfig, base_changing_length_),
  PROTOBUF_FIELD_OFFSET(::apollo::routing::RoutingConfig, topic_config_),
  1,
  2,
  3,
  4,
  5,
  6,
  0,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, -1, sizeof(::apollo::routing::TopicConfig)},
  { 10, 23, -1, sizeof(::apollo::routing::RoutingConfig)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::routing::_TopicConfig_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::routing::_RoutingConfig_default_instance_),
};

const char descriptor_table_protodef_modules_2frouting_2fproto_2frouting_5fconfig_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n*modules/routing/proto/routing_config.p"
  "roto\022\016apollo.routing\"U\n\013TopicConfig\022\036\n\026r"
  "outing_response_topic\030\001 \001(\t\022&\n\036routing_r"
  "esponse_history_topic\030\002 \001(\t\"\332\001\n\rRoutingC"
  "onfig\022\022\n\nbase_speed\030\001 \001(\001\022\031\n\021left_turn_p"
  "enalty\030\002 \001(\001\022\032\n\022right_turn_penalty\030\003 \001(\001"
  "\022\025\n\ruturn_penalty\030\004 \001(\001\022\026\n\016change_penalt"
  "y\030\005 \001(\001\022\034\n\024base_changing_length\030\006 \001(\001\0221\n"
  "\014topic_config\030\007 \001(\0132\033.apollo.routing.Top"
  "icConfig"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2frouting_2fproto_2frouting_5fconfig_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2frouting_2fproto_2frouting_5fconfig_2eproto = {
  false, false, 368, descriptor_table_protodef_modules_2frouting_2fproto_2frouting_5fconfig_2eproto, "modules/routing/proto/routing_config.proto", 
  &descriptor_table_modules_2frouting_2fproto_2frouting_5fconfig_2eproto_once, nullptr, 0, 2,
  schemas, file_default_instances, TableStruct_modules_2frouting_2fproto_2frouting_5fconfig_2eproto::offsets,
  file_level_metadata_modules_2frouting_2fproto_2frouting_5fconfig_2eproto, file_level_enum_descriptors_modules_2frouting_2fproto_2frouting_5fconfig_2eproto, file_level_service_descriptors_modules_2frouting_2fproto_2frouting_5fconfig_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2frouting_2fproto_2frouting_5fconfig_2eproto_getter() {
  return &descriptor_table_modules_2frouting_2fproto_2frouting_5fconfig_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2frouting_2fproto_2frouting_5fconfig_2eproto(&descriptor_table_modules_2frouting_2fproto_2frouting_5fconfig_2eproto);
namespace apollo {
namespace routing {

// ===================================================================

class TopicConfig::_Internal {
 public:
  using HasBits = decltype(std::declval<TopicConfig>()._has_bits_);
  static void set_has_routing_response_topic(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_routing_response_history_topic(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

TopicConfig::TopicConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.routing.TopicConfig)
}
TopicConfig::TopicConfig(const TopicConfig& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  routing_response_topic_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_routing_response_topic()) {
    routing_response_topic_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_routing_response_topic(), 
      GetArenaForAllocation());
  }
  routing_response_history_topic_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_routing_response_history_topic()) {
    routing_response_history_topic_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_routing_response_history_topic(), 
      GetArenaForAllocation());
  }
  // @@protoc_insertion_point(copy_constructor:apollo.routing.TopicConfig)
}

void TopicConfig::SharedCtor() {
routing_response_topic_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
routing_response_history_topic_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

TopicConfig::~TopicConfig() {
  // @@protoc_insertion_point(destructor:apollo.routing.TopicConfig)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void TopicConfig::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  routing_response_topic_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  routing_response_history_topic_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void TopicConfig::ArenaDtor(void* object) {
  TopicConfig* _this = reinterpret_cast< TopicConfig* >(object);
  (void)_this;
}
void TopicConfig::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void TopicConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void TopicConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.routing.TopicConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      routing_response_topic_.ClearNonDefaultToEmpty();
    }
    if (cached_has_bits & 0x00000002u) {
      routing_response_history_topic_.ClearNonDefaultToEmpty();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* TopicConfig::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional string routing_response_topic = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          auto str = _internal_mutable_routing_response_topic();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.routing.TopicConfig.routing_response_topic");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional string routing_response_history_topic = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          auto str = _internal_mutable_routing_response_history_topic();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.routing.TopicConfig.routing_response_history_topic");
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

::PROTOBUF_NAMESPACE_ID::uint8* TopicConfig::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.routing.TopicConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string routing_response_topic = 1;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_routing_response_topic().data(), static_cast<int>(this->_internal_routing_response_topic().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.routing.TopicConfig.routing_response_topic");
    target = stream->WriteStringMaybeAliased(
        1, this->_internal_routing_response_topic(), target);
  }

  // optional string routing_response_history_topic = 2;
  if (cached_has_bits & 0x00000002u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_routing_response_history_topic().data(), static_cast<int>(this->_internal_routing_response_history_topic().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.routing.TopicConfig.routing_response_history_topic");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_routing_response_history_topic(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.routing.TopicConfig)
  return target;
}

size_t TopicConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.routing.TopicConfig)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    // optional string routing_response_topic = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_routing_response_topic());
    }

    // optional string routing_response_history_topic = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_routing_response_history_topic());
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData TopicConfig::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    TopicConfig::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*TopicConfig::GetClassData() const { return &_class_data_; }

void TopicConfig::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<TopicConfig *>(to)->MergeFrom(
      static_cast<const TopicConfig &>(from));
}


void TopicConfig::MergeFrom(const TopicConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.routing.TopicConfig)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_set_routing_response_topic(from._internal_routing_response_topic());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_set_routing_response_history_topic(from._internal_routing_response_history_topic());
    }
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void TopicConfig::CopyFrom(const TopicConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.routing.TopicConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool TopicConfig::IsInitialized() const {
  return true;
}

void TopicConfig::InternalSwap(TopicConfig* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &routing_response_topic_, lhs_arena,
      &other->routing_response_topic_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &routing_response_history_topic_, lhs_arena,
      &other->routing_response_history_topic_, rhs_arena
  );
}

::PROTOBUF_NAMESPACE_ID::Metadata TopicConfig::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2frouting_2fproto_2frouting_5fconfig_2eproto_getter, &descriptor_table_modules_2frouting_2fproto_2frouting_5fconfig_2eproto_once,
      file_level_metadata_modules_2frouting_2fproto_2frouting_5fconfig_2eproto[0]);
}

// ===================================================================

class RoutingConfig::_Internal {
 public:
  using HasBits = decltype(std::declval<RoutingConfig>()._has_bits_);
  static void set_has_base_speed(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_left_turn_penalty(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_right_turn_penalty(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_uturn_penalty(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static void set_has_change_penalty(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static void set_has_base_changing_length(HasBits* has_bits) {
    (*has_bits)[0] |= 64u;
  }
  static const ::apollo::routing::TopicConfig& topic_config(const RoutingConfig* msg);
  static void set_has_topic_config(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

const ::apollo::routing::TopicConfig&
RoutingConfig::_Internal::topic_config(const RoutingConfig* msg) {
  return *msg->topic_config_;
}
RoutingConfig::RoutingConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.routing.RoutingConfig)
}
RoutingConfig::RoutingConfig(const RoutingConfig& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_topic_config()) {
    topic_config_ = new ::apollo::routing::TopicConfig(*from.topic_config_);
  } else {
    topic_config_ = nullptr;
  }
  ::memcpy(&base_speed_, &from.base_speed_,
    static_cast<size_t>(reinterpret_cast<char*>(&base_changing_length_) -
    reinterpret_cast<char*>(&base_speed_)) + sizeof(base_changing_length_));
  // @@protoc_insertion_point(copy_constructor:apollo.routing.RoutingConfig)
}

void RoutingConfig::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&topic_config_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&base_changing_length_) -
    reinterpret_cast<char*>(&topic_config_)) + sizeof(base_changing_length_));
}

RoutingConfig::~RoutingConfig() {
  // @@protoc_insertion_point(destructor:apollo.routing.RoutingConfig)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void RoutingConfig::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete topic_config_;
}

void RoutingConfig::ArenaDtor(void* object) {
  RoutingConfig* _this = reinterpret_cast< RoutingConfig* >(object);
  (void)_this;
}
void RoutingConfig::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void RoutingConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void RoutingConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.routing.RoutingConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(topic_config_ != nullptr);
    topic_config_->Clear();
  }
  if (cached_has_bits & 0x0000007eu) {
    ::memset(&base_speed_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&base_changing_length_) -
        reinterpret_cast<char*>(&base_speed_)) + sizeof(base_changing_length_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* RoutingConfig::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional double base_speed = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          _Internal::set_has_base_speed(&has_bits);
          base_speed_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional double left_turn_penalty = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 17)) {
          _Internal::set_has_left_turn_penalty(&has_bits);
          left_turn_penalty_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional double right_turn_penalty = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 25)) {
          _Internal::set_has_right_turn_penalty(&has_bits);
          right_turn_penalty_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional double uturn_penalty = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 33)) {
          _Internal::set_has_uturn_penalty(&has_bits);
          uturn_penalty_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional double change_penalty = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 41)) {
          _Internal::set_has_change_penalty(&has_bits);
          change_penalty_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional double base_changing_length = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 49)) {
          _Internal::set_has_base_changing_length(&has_bits);
          base_changing_length_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional .apollo.routing.TopicConfig topic_config = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 58)) {
          ptr = ctx->ParseMessage(_internal_mutable_topic_config(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* RoutingConfig::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.routing.RoutingConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double base_speed = 1;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_base_speed(), target);
  }

  // optional double left_turn_penalty = 2;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_left_turn_penalty(), target);
  }

  // optional double right_turn_penalty = 3;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(3, this->_internal_right_turn_penalty(), target);
  }

  // optional double uturn_penalty = 4;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(4, this->_internal_uturn_penalty(), target);
  }

  // optional double change_penalty = 5;
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(5, this->_internal_change_penalty(), target);
  }

  // optional double base_changing_length = 6;
  if (cached_has_bits & 0x00000040u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(6, this->_internal_base_changing_length(), target);
  }

  // optional .apollo.routing.TopicConfig topic_config = 7;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        7, _Internal::topic_config(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.routing.RoutingConfig)
  return target;
}

size_t RoutingConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.routing.RoutingConfig)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000007fu) {
    // optional .apollo.routing.TopicConfig topic_config = 7;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *topic_config_);
    }

    // optional double base_speed = 1;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 + 8;
    }

    // optional double left_turn_penalty = 2;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 + 8;
    }

    // optional double right_turn_penalty = 3;
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 + 8;
    }

    // optional double uturn_penalty = 4;
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 + 8;
    }

    // optional double change_penalty = 5;
    if (cached_has_bits & 0x00000020u) {
      total_size += 1 + 8;
    }

    // optional double base_changing_length = 6;
    if (cached_has_bits & 0x00000040u) {
      total_size += 1 + 8;
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData RoutingConfig::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    RoutingConfig::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*RoutingConfig::GetClassData() const { return &_class_data_; }

void RoutingConfig::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<RoutingConfig *>(to)->MergeFrom(
      static_cast<const RoutingConfig &>(from));
}


void RoutingConfig::MergeFrom(const RoutingConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.routing.RoutingConfig)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000007fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_topic_config()->::apollo::routing::TopicConfig::MergeFrom(from._internal_topic_config());
    }
    if (cached_has_bits & 0x00000002u) {
      base_speed_ = from.base_speed_;
    }
    if (cached_has_bits & 0x00000004u) {
      left_turn_penalty_ = from.left_turn_penalty_;
    }
    if (cached_has_bits & 0x00000008u) {
      right_turn_penalty_ = from.right_turn_penalty_;
    }
    if (cached_has_bits & 0x00000010u) {
      uturn_penalty_ = from.uturn_penalty_;
    }
    if (cached_has_bits & 0x00000020u) {
      change_penalty_ = from.change_penalty_;
    }
    if (cached_has_bits & 0x00000040u) {
      base_changing_length_ = from.base_changing_length_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void RoutingConfig::CopyFrom(const RoutingConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.routing.RoutingConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RoutingConfig::IsInitialized() const {
  return true;
}

void RoutingConfig::InternalSwap(RoutingConfig* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(RoutingConfig, base_changing_length_)
      + sizeof(RoutingConfig::base_changing_length_)
      - PROTOBUF_FIELD_OFFSET(RoutingConfig, topic_config_)>(
          reinterpret_cast<char*>(&topic_config_),
          reinterpret_cast<char*>(&other->topic_config_));
}

::PROTOBUF_NAMESPACE_ID::Metadata RoutingConfig::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2frouting_2fproto_2frouting_5fconfig_2eproto_getter, &descriptor_table_modules_2frouting_2fproto_2frouting_5fconfig_2eproto_once,
      file_level_metadata_modules_2frouting_2fproto_2frouting_5fconfig_2eproto[1]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace routing
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::routing::TopicConfig* Arena::CreateMaybeMessage< ::apollo::routing::TopicConfig >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::routing::TopicConfig >(arena);
}
template<> PROTOBUF_NOINLINE ::apollo::routing::RoutingConfig* Arena::CreateMaybeMessage< ::apollo::routing::RoutingConfig >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::routing::RoutingConfig >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
