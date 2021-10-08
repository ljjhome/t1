// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/lidar/app/proto/lidar_obstacle_tracking_config.proto

#include "modules/perception/lidar/app/proto/lidar_obstacle_tracking_config.pb.h"

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
namespace lidar {
constexpr LidarObstacleTrackingConfig::LidarObstacleTrackingConfig(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : multi_target_tracker_(nullptr)
  , frame_classifier_(nullptr)
  , fusion_classifier_(nullptr){}
struct LidarObstacleTrackingConfigDefaultTypeInternal {
  constexpr LidarObstacleTrackingConfigDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~LidarObstacleTrackingConfigDefaultTypeInternal() {}
  union {
    LidarObstacleTrackingConfig _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT LidarObstacleTrackingConfigDefaultTypeInternal _LidarObstacleTrackingConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::LidarObstacleTrackingConfig, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::LidarObstacleTrackingConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::LidarObstacleTrackingConfig, multi_target_tracker_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::LidarObstacleTrackingConfig, frame_classifier_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::LidarObstacleTrackingConfig, fusion_classifier_),
  0,
  1,
  2,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, -1, sizeof(::apollo::perception::lidar::LidarObstacleTrackingConfig)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::perception::lidar::_LidarObstacleTrackingConfig_default_instance_),
};

const char descriptor_table_protodef_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\nGmodules/perception/lidar/app/proto/lid"
  "ar_obstacle_tracking_config.proto\022\027apoll"
  "o.perception.lidar\"\253\001\n\033LidarObstacleTrac"
  "kingConfig\0225\n\024multi_target_tracker\030\001 \001(\t"
  ":\027DummyMultiTargetTracker\022)\n\020frame_class"
  "ifier\030\002 \001(\t:\017DummyClassifier\022*\n\021fusion_c"
  "lassifier\030\003 \001(\t:\017DummyClassifier"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto = {
  false, false, 272, descriptor_table_protodef_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto, "modules/perception/lidar/app/proto/lidar_obstacle_tracking_config.proto", 
  &descriptor_table_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto::offsets,
  file_level_metadata_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto, file_level_enum_descriptors_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto, file_level_service_descriptors_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto_getter() {
  return &descriptor_table_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto(&descriptor_table_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto);
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

class LidarObstacleTrackingConfig::_Internal {
 public:
  using HasBits = decltype(std::declval<LidarObstacleTrackingConfig>()._has_bits_);
  static void set_has_multi_target_tracker(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_frame_classifier(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_fusion_classifier(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::PROTOBUF_NAMESPACE_ID::internal::LazyString LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_multi_target_tracker_{{{"DummyMultiTargetTracker", 23}}, {nullptr}};
const ::PROTOBUF_NAMESPACE_ID::internal::LazyString LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_frame_classifier_{{{"DummyClassifier", 15}}, {nullptr}};
const ::PROTOBUF_NAMESPACE_ID::internal::LazyString LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_fusion_classifier_{{{"DummyClassifier", 15}}, {nullptr}};
LidarObstacleTrackingConfig::LidarObstacleTrackingConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.perception.lidar.LidarObstacleTrackingConfig)
}
LidarObstacleTrackingConfig::LidarObstacleTrackingConfig(const LidarObstacleTrackingConfig& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  multi_target_tracker_.UnsafeSetDefault(nullptr);
  if (from._internal_has_multi_target_tracker()) {
    multi_target_tracker_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::NonEmptyDefault{}, from._internal_multi_target_tracker(), 
      GetArenaForAllocation());
  }
  frame_classifier_.UnsafeSetDefault(nullptr);
  if (from._internal_has_frame_classifier()) {
    frame_classifier_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::NonEmptyDefault{}, from._internal_frame_classifier(), 
      GetArenaForAllocation());
  }
  fusion_classifier_.UnsafeSetDefault(nullptr);
  if (from._internal_has_fusion_classifier()) {
    fusion_classifier_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::NonEmptyDefault{}, from._internal_fusion_classifier(), 
      GetArenaForAllocation());
  }
  // @@protoc_insertion_point(copy_constructor:apollo.perception.lidar.LidarObstacleTrackingConfig)
}

void LidarObstacleTrackingConfig::SharedCtor() {
multi_target_tracker_.UnsafeSetDefault(nullptr);
frame_classifier_.UnsafeSetDefault(nullptr);
fusion_classifier_.UnsafeSetDefault(nullptr);
}

LidarObstacleTrackingConfig::~LidarObstacleTrackingConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.lidar.LidarObstacleTrackingConfig)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void LidarObstacleTrackingConfig::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  multi_target_tracker_.DestroyNoArena(nullptr);
  frame_classifier_.DestroyNoArena(nullptr);
  fusion_classifier_.DestroyNoArena(nullptr);
}

void LidarObstacleTrackingConfig::ArenaDtor(void* object) {
  LidarObstacleTrackingConfig* _this = reinterpret_cast< LidarObstacleTrackingConfig* >(object);
  (void)_this;
}
void LidarObstacleTrackingConfig::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void LidarObstacleTrackingConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void LidarObstacleTrackingConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.lidar.LidarObstacleTrackingConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      multi_target_tracker_.ClearToDefault(::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_multi_target_tracker_, GetArenaForAllocation());
       }
    if (cached_has_bits & 0x00000002u) {
      frame_classifier_.ClearToDefault(::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_frame_classifier_, GetArenaForAllocation());
       }
    if (cached_has_bits & 0x00000004u) {
      fusion_classifier_.ClearToDefault(::apollo::perception::lidar::LidarObstacleTrackingConfig::_i_give_permission_to_break_this_code_default_fusion_classifier_, GetArenaForAllocation());
       }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* LidarObstacleTrackingConfig::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional string multi_target_tracker = 1 [default = "DummyMultiTargetTracker"];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          auto str = _internal_mutable_multi_target_tracker();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.lidar.LidarObstacleTrackingConfig.multi_target_tracker");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional string frame_classifier = 2 [default = "DummyClassifier"];
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          auto str = _internal_mutable_frame_classifier();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.lidar.LidarObstacleTrackingConfig.frame_classifier");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional string fusion_classifier = 3 [default = "DummyClassifier"];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          auto str = _internal_mutable_fusion_classifier();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.lidar.LidarObstacleTrackingConfig.fusion_classifier");
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

::PROTOBUF_NAMESPACE_ID::uint8* LidarObstacleTrackingConfig::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.lidar.LidarObstacleTrackingConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string multi_target_tracker = 1 [default = "DummyMultiTargetTracker"];
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_multi_target_tracker().data(), static_cast<int>(this->_internal_multi_target_tracker().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.lidar.LidarObstacleTrackingConfig.multi_target_tracker");
    target = stream->WriteStringMaybeAliased(
        1, this->_internal_multi_target_tracker(), target);
  }

  // optional string frame_classifier = 2 [default = "DummyClassifier"];
  if (cached_has_bits & 0x00000002u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_frame_classifier().data(), static_cast<int>(this->_internal_frame_classifier().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.lidar.LidarObstacleTrackingConfig.frame_classifier");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_frame_classifier(), target);
  }

  // optional string fusion_classifier = 3 [default = "DummyClassifier"];
  if (cached_has_bits & 0x00000004u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_fusion_classifier().data(), static_cast<int>(this->_internal_fusion_classifier().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.lidar.LidarObstacleTrackingConfig.fusion_classifier");
    target = stream->WriteStringMaybeAliased(
        3, this->_internal_fusion_classifier(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.lidar.LidarObstacleTrackingConfig)
  return target;
}

size_t LidarObstacleTrackingConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.lidar.LidarObstacleTrackingConfig)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional string multi_target_tracker = 1 [default = "DummyMultiTargetTracker"];
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_multi_target_tracker());
    }

    // optional string frame_classifier = 2 [default = "DummyClassifier"];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_frame_classifier());
    }

    // optional string fusion_classifier = 3 [default = "DummyClassifier"];
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_fusion_classifier());
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData LidarObstacleTrackingConfig::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    LidarObstacleTrackingConfig::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*LidarObstacleTrackingConfig::GetClassData() const { return &_class_data_; }

void LidarObstacleTrackingConfig::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<LidarObstacleTrackingConfig *>(to)->MergeFrom(
      static_cast<const LidarObstacleTrackingConfig &>(from));
}


void LidarObstacleTrackingConfig::MergeFrom(const LidarObstacleTrackingConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.lidar.LidarObstacleTrackingConfig)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_set_multi_target_tracker(from._internal_multi_target_tracker());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_set_frame_classifier(from._internal_frame_classifier());
    }
    if (cached_has_bits & 0x00000004u) {
      _internal_set_fusion_classifier(from._internal_fusion_classifier());
    }
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void LidarObstacleTrackingConfig::CopyFrom(const LidarObstacleTrackingConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.lidar.LidarObstacleTrackingConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LidarObstacleTrackingConfig::IsInitialized() const {
  return true;
}

void LidarObstacleTrackingConfig::InternalSwap(LidarObstacleTrackingConfig* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      nullptr,
      &multi_target_tracker_, lhs_arena,
      &other->multi_target_tracker_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      nullptr,
      &frame_classifier_, lhs_arena,
      &other->frame_classifier_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      nullptr,
      &fusion_classifier_, lhs_arena,
      &other->fusion_classifier_, rhs_arena
  );
}

::PROTOBUF_NAMESPACE_ID::Metadata LidarObstacleTrackingConfig::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto_getter, &descriptor_table_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto_once,
      file_level_metadata_modules_2fperception_2flidar_2fapp_2fproto_2flidar_5fobstacle_5ftracking_5fconfig_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::perception::lidar::LidarObstacleTrackingConfig* Arena::CreateMaybeMessage< ::apollo::perception::lidar::LidarObstacleTrackingConfig >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::perception::lidar::LidarObstacleTrackingConfig >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
