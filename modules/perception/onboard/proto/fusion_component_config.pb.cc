// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/onboard/proto/fusion_component_config.proto

#include "modules/perception/onboard/proto/fusion_component_config.pb.h"

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
namespace perception {
namespace onboard {
constexpr FusionComponentConfig::FusionComponentConfig(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : fusion_main_sensors_()
  , fusion_name_(nullptr)
  , fusion_method_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , output_obstacles_channel_name_(nullptr)
  , output_viz_fused_content_channel_name_(nullptr)
  , radius_for_roi_object_check_(0)
  , object_in_roi_check_(false){}
struct FusionComponentConfigDefaultTypeInternal {
  constexpr FusionComponentConfigDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~FusionComponentConfigDefaultTypeInternal() {}
  union {
    FusionComponentConfig _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT FusionComponentConfigDefaultTypeInternal _FusionComponentConfig_default_instance_;
}  // namespace onboard
}  // namespace perception
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::perception::onboard::FusionComponentConfig, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::onboard::FusionComponentConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::perception::onboard::FusionComponentConfig, fusion_name_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::onboard::FusionComponentConfig, fusion_method_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::onboard::FusionComponentConfig, fusion_main_sensors_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::onboard::FusionComponentConfig, object_in_roi_check_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::onboard::FusionComponentConfig, radius_for_roi_object_check_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::onboard::FusionComponentConfig, output_obstacles_channel_name_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::onboard::FusionComponentConfig, output_viz_fused_content_channel_name_),
  0,
  1,
  ~0u,
  5,
  4,
  2,
  3,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 13, -1, sizeof(::apollo::perception::onboard::FusionComponentConfig)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::perception::onboard::_FusionComponentConfig_default_instance_),
};

const char descriptor_table_protodef_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n>modules/perception/onboard/proto/fusio"
  "n_component_config.proto\022\031apollo.percept"
  "ion.onboard\"\340\002\n\025FusionComponentConfig\022.\n"
  "\013fusion_name\030\001 \001(\t:\031ObstacleMultiSensorF"
  "usion\022\025\n\rfusion_method\030\002 \001(\t\022\033\n\023fusion_m"
  "ain_sensors\030\003 \003(\t\022\033\n\023object_in_roi_check"
  "\030\004 \001(\010\022#\n\033radius_for_roi_object_check\030\005 "
  "\001(\001\022D\n\035output_obstacles_channel_name\030\006 \001"
  "(\t:\035/perception/vehicle/obstacles\022[\n%out"
  "put_viz_fused_content_channel_name\030\007 \001(\t"
  ":,/perception/inner/visualization/FusedO"
  "bjects"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto = {
  false, false, 446, descriptor_table_protodef_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto, "modules/perception/onboard/proto/fusion_component_config.proto", 
  &descriptor_table_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto::offsets,
  file_level_metadata_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto, file_level_enum_descriptors_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto, file_level_service_descriptors_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto_getter() {
  return &descriptor_table_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto(&descriptor_table_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto);
namespace apollo {
namespace perception {
namespace onboard {

// ===================================================================

class FusionComponentConfig::_Internal {
 public:
  using HasBits = decltype(std::declval<FusionComponentConfig>()._has_bits_);
  static void set_has_fusion_name(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_fusion_method(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_object_in_roi_check(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static void set_has_radius_for_roi_object_check(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static void set_has_output_obstacles_channel_name(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_output_viz_fused_content_channel_name(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
};

const ::PROTOBUF_NAMESPACE_ID::internal::LazyString FusionComponentConfig::_i_give_permission_to_break_this_code_default_fusion_name_{{{"ObstacleMultiSensorFusion", 25}}, {nullptr}};
const ::PROTOBUF_NAMESPACE_ID::internal::LazyString FusionComponentConfig::_i_give_permission_to_break_this_code_default_output_obstacles_channel_name_{{{"/perception/vehicle/obstacles", 29}}, {nullptr}};
const ::PROTOBUF_NAMESPACE_ID::internal::LazyString FusionComponentConfig::_i_give_permission_to_break_this_code_default_output_viz_fused_content_channel_name_{{{"/perception/inner/visualization/FusedObjects", 44}}, {nullptr}};
FusionComponentConfig::FusionComponentConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  fusion_main_sensors_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.perception.onboard.FusionComponentConfig)
}
FusionComponentConfig::FusionComponentConfig(const FusionComponentConfig& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      fusion_main_sensors_(from.fusion_main_sensors_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  fusion_name_.UnsafeSetDefault(nullptr);
  if (from._internal_has_fusion_name()) {
    fusion_name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::NonEmptyDefault{}, from._internal_fusion_name(), 
      GetArenaForAllocation());
  }
  fusion_method_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_fusion_method()) {
    fusion_method_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_fusion_method(), 
      GetArenaForAllocation());
  }
  output_obstacles_channel_name_.UnsafeSetDefault(nullptr);
  if (from._internal_has_output_obstacles_channel_name()) {
    output_obstacles_channel_name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::NonEmptyDefault{}, from._internal_output_obstacles_channel_name(), 
      GetArenaForAllocation());
  }
  output_viz_fused_content_channel_name_.UnsafeSetDefault(nullptr);
  if (from._internal_has_output_viz_fused_content_channel_name()) {
    output_viz_fused_content_channel_name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::NonEmptyDefault{}, from._internal_output_viz_fused_content_channel_name(), 
      GetArenaForAllocation());
  }
  ::memcpy(&radius_for_roi_object_check_, &from.radius_for_roi_object_check_,
    static_cast<size_t>(reinterpret_cast<char*>(&object_in_roi_check_) -
    reinterpret_cast<char*>(&radius_for_roi_object_check_)) + sizeof(object_in_roi_check_));
  // @@protoc_insertion_point(copy_constructor:apollo.perception.onboard.FusionComponentConfig)
}

void FusionComponentConfig::SharedCtor() {
fusion_name_.UnsafeSetDefault(nullptr);
fusion_method_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
output_obstacles_channel_name_.UnsafeSetDefault(nullptr);
output_viz_fused_content_channel_name_.UnsafeSetDefault(nullptr);
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&radius_for_roi_object_check_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&object_in_roi_check_) -
    reinterpret_cast<char*>(&radius_for_roi_object_check_)) + sizeof(object_in_roi_check_));
}

FusionComponentConfig::~FusionComponentConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.onboard.FusionComponentConfig)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void FusionComponentConfig::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  fusion_name_.DestroyNoArena(nullptr);
  fusion_method_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  output_obstacles_channel_name_.DestroyNoArena(nullptr);
  output_viz_fused_content_channel_name_.DestroyNoArena(nullptr);
}

void FusionComponentConfig::ArenaDtor(void* object) {
  FusionComponentConfig* _this = reinterpret_cast< FusionComponentConfig* >(object);
  (void)_this;
}
void FusionComponentConfig::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void FusionComponentConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void FusionComponentConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.onboard.FusionComponentConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  fusion_main_sensors_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      fusion_name_.ClearToDefault(::apollo::perception::onboard::FusionComponentConfig::_i_give_permission_to_break_this_code_default_fusion_name_, GetArenaForAllocation());
       }
    if (cached_has_bits & 0x00000002u) {
      fusion_method_.ClearNonDefaultToEmpty();
    }
    if (cached_has_bits & 0x00000004u) {
      output_obstacles_channel_name_.ClearToDefault(::apollo::perception::onboard::FusionComponentConfig::_i_give_permission_to_break_this_code_default_output_obstacles_channel_name_, GetArenaForAllocation());
       }
    if (cached_has_bits & 0x00000008u) {
      output_viz_fused_content_channel_name_.ClearToDefault(::apollo::perception::onboard::FusionComponentConfig::_i_give_permission_to_break_this_code_default_output_viz_fused_content_channel_name_, GetArenaForAllocation());
       }
  }
  if (cached_has_bits & 0x00000030u) {
    ::memset(&radius_for_roi_object_check_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&object_in_roi_check_) -
        reinterpret_cast<char*>(&radius_for_roi_object_check_)) + sizeof(object_in_roi_check_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* FusionComponentConfig::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional string fusion_name = 1 [default = "ObstacleMultiSensorFusion"];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          auto str = _internal_mutable_fusion_name();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.onboard.FusionComponentConfig.fusion_name");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional string fusion_method = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          auto str = _internal_mutable_fusion_method();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.onboard.FusionComponentConfig.fusion_method");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated string fusion_main_sensors = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr -= 1;
          do {
            ptr += 1;
            auto str = _internal_add_fusion_main_sensors();
            ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
            #ifndef NDEBUG
            ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.onboard.FusionComponentConfig.fusion_main_sensors");
            #endif  // !NDEBUG
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<26>(ptr));
        } else
          goto handle_unusual;
        continue;
      // optional bool object_in_roi_check = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 32)) {
          _Internal::set_has_object_in_roi_check(&has_bits);
          object_in_roi_check_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional double radius_for_roi_object_check = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 41)) {
          _Internal::set_has_radius_for_roi_object_check(&has_bits);
          radius_for_roi_object_check_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional string output_obstacles_channel_name = 6 [default = "/perception/vehicle/obstacles"];
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 50)) {
          auto str = _internal_mutable_output_obstacles_channel_name();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.onboard.FusionComponentConfig.output_obstacles_channel_name");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional string output_viz_fused_content_channel_name = 7 [default = "/perception/inner/visualization/FusedObjects"];
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 58)) {
          auto str = _internal_mutable_output_viz_fused_content_channel_name();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.onboard.FusionComponentConfig.output_viz_fused_content_channel_name");
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

::PROTOBUF_NAMESPACE_ID::uint8* FusionComponentConfig::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.onboard.FusionComponentConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string fusion_name = 1 [default = "ObstacleMultiSensorFusion"];
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_fusion_name().data(), static_cast<int>(this->_internal_fusion_name().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.onboard.FusionComponentConfig.fusion_name");
    target = stream->WriteStringMaybeAliased(
        1, this->_internal_fusion_name(), target);
  }

  // optional string fusion_method = 2;
  if (cached_has_bits & 0x00000002u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_fusion_method().data(), static_cast<int>(this->_internal_fusion_method().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.onboard.FusionComponentConfig.fusion_method");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_fusion_method(), target);
  }

  // repeated string fusion_main_sensors = 3;
  for (int i = 0, n = this->_internal_fusion_main_sensors_size(); i < n; i++) {
    const auto& s = this->_internal_fusion_main_sensors(i);
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      s.data(), static_cast<int>(s.length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.onboard.FusionComponentConfig.fusion_main_sensors");
    target = stream->WriteString(3, s, target);
  }

  // optional bool object_in_roi_check = 4;
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(4, this->_internal_object_in_roi_check(), target);
  }

  // optional double radius_for_roi_object_check = 5;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(5, this->_internal_radius_for_roi_object_check(), target);
  }

  // optional string output_obstacles_channel_name = 6 [default = "/perception/vehicle/obstacles"];
  if (cached_has_bits & 0x00000004u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_output_obstacles_channel_name().data(), static_cast<int>(this->_internal_output_obstacles_channel_name().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.onboard.FusionComponentConfig.output_obstacles_channel_name");
    target = stream->WriteStringMaybeAliased(
        6, this->_internal_output_obstacles_channel_name(), target);
  }

  // optional string output_viz_fused_content_channel_name = 7 [default = "/perception/inner/visualization/FusedObjects"];
  if (cached_has_bits & 0x00000008u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_output_viz_fused_content_channel_name().data(), static_cast<int>(this->_internal_output_viz_fused_content_channel_name().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.onboard.FusionComponentConfig.output_viz_fused_content_channel_name");
    target = stream->WriteStringMaybeAliased(
        7, this->_internal_output_viz_fused_content_channel_name(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.onboard.FusionComponentConfig)
  return target;
}

size_t FusionComponentConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.onboard.FusionComponentConfig)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated string fusion_main_sensors = 3;
  total_size += 1 *
      ::PROTOBUF_NAMESPACE_ID::internal::FromIntSize(fusion_main_sensors_.size());
  for (int i = 0, n = fusion_main_sensors_.size(); i < n; i++) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
      fusion_main_sensors_.Get(i));
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000003fu) {
    // optional string fusion_name = 1 [default = "ObstacleMultiSensorFusion"];
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_fusion_name());
    }

    // optional string fusion_method = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_fusion_method());
    }

    // optional string output_obstacles_channel_name = 6 [default = "/perception/vehicle/obstacles"];
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_output_obstacles_channel_name());
    }

    // optional string output_viz_fused_content_channel_name = 7 [default = "/perception/inner/visualization/FusedObjects"];
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_output_viz_fused_content_channel_name());
    }

    // optional double radius_for_roi_object_check = 5;
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 + 8;
    }

    // optional bool object_in_roi_check = 4;
    if (cached_has_bits & 0x00000020u) {
      total_size += 1 + 1;
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData FusionComponentConfig::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    FusionComponentConfig::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*FusionComponentConfig::GetClassData() const { return &_class_data_; }

void FusionComponentConfig::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<FusionComponentConfig *>(to)->MergeFrom(
      static_cast<const FusionComponentConfig &>(from));
}


void FusionComponentConfig::MergeFrom(const FusionComponentConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.onboard.FusionComponentConfig)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  fusion_main_sensors_.MergeFrom(from.fusion_main_sensors_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000003fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_set_fusion_name(from._internal_fusion_name());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_set_fusion_method(from._internal_fusion_method());
    }
    if (cached_has_bits & 0x00000004u) {
      _internal_set_output_obstacles_channel_name(from._internal_output_obstacles_channel_name());
    }
    if (cached_has_bits & 0x00000008u) {
      _internal_set_output_viz_fused_content_channel_name(from._internal_output_viz_fused_content_channel_name());
    }
    if (cached_has_bits & 0x00000010u) {
      radius_for_roi_object_check_ = from.radius_for_roi_object_check_;
    }
    if (cached_has_bits & 0x00000020u) {
      object_in_roi_check_ = from.object_in_roi_check_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void FusionComponentConfig::CopyFrom(const FusionComponentConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.onboard.FusionComponentConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool FusionComponentConfig::IsInitialized() const {
  return true;
}

void FusionComponentConfig::InternalSwap(FusionComponentConfig* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  fusion_main_sensors_.InternalSwap(&other->fusion_main_sensors_);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      nullptr,
      &fusion_name_, lhs_arena,
      &other->fusion_name_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &fusion_method_, lhs_arena,
      &other->fusion_method_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      nullptr,
      &output_obstacles_channel_name_, lhs_arena,
      &other->output_obstacles_channel_name_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      nullptr,
      &output_viz_fused_content_channel_name_, lhs_arena,
      &other->output_viz_fused_content_channel_name_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(FusionComponentConfig, object_in_roi_check_)
      + sizeof(FusionComponentConfig::object_in_roi_check_)
      - PROTOBUF_FIELD_OFFSET(FusionComponentConfig, radius_for_roi_object_check_)>(
          reinterpret_cast<char*>(&radius_for_roi_object_check_),
          reinterpret_cast<char*>(&other->radius_for_roi_object_check_));
}

::PROTOBUF_NAMESPACE_ID::Metadata FusionComponentConfig::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto_getter, &descriptor_table_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto_once,
      file_level_metadata_modules_2fperception_2fonboard_2fproto_2ffusion_5fcomponent_5fconfig_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace onboard
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::perception::onboard::FusionComponentConfig* Arena::CreateMaybeMessage< ::apollo::perception::onboard::FusionComponentConfig >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::perception::onboard::FusionComponentConfig >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>