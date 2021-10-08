// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/proto/map_manager_config.proto

#include "modules/perception/proto/map_manager_config.pb.h"

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
constexpr MapManagerConfig::MapManagerConfig(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : lane_range_(0)
  , max_depth_(0)
  , update_pose_(false)
  , roi_search_distance_(80){}
struct MapManagerConfigDefaultTypeInternal {
  constexpr MapManagerConfigDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~MapManagerConfigDefaultTypeInternal() {}
  union {
    MapManagerConfig _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT MapManagerConfigDefaultTypeInternal _MapManagerConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::MapManagerConfig, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::MapManagerConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::MapManagerConfig, update_pose_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::MapManagerConfig, roi_search_distance_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::MapManagerConfig, lane_range_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::MapManagerConfig, max_depth_),
  2,
  3,
  0,
  1,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 10, -1, sizeof(::apollo::perception::lidar::MapManagerConfig)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::perception::lidar::_MapManagerConfig_default_instance_),
};

const char descriptor_table_protodef_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n1modules/perception/proto/map_manager_c"
  "onfig.proto\022\027apollo.perception.lidar\"v\n\020"
  "MapManagerConfig\022\032\n\013update_pose\030\001 \001(\010:\005f"
  "alse\022\037\n\023roi_search_distance\030\002 \001(\001:\00280\022\022\n"
  "\nlane_range\030\003 \001(\001\022\021\n\tmax_depth\030\004 \001(\001"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto = {
  false, false, 196, descriptor_table_protodef_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto, "modules/perception/proto/map_manager_config.proto", 
  &descriptor_table_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto::offsets,
  file_level_metadata_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto, file_level_enum_descriptors_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto, file_level_service_descriptors_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto_getter() {
  return &descriptor_table_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto(&descriptor_table_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto);
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

class MapManagerConfig::_Internal {
 public:
  using HasBits = decltype(std::declval<MapManagerConfig>()._has_bits_);
  static void set_has_update_pose(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_roi_search_distance(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_lane_range(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_max_depth(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

MapManagerConfig::MapManagerConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.perception.lidar.MapManagerConfig)
}
MapManagerConfig::MapManagerConfig(const MapManagerConfig& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&lane_range_, &from.lane_range_,
    static_cast<size_t>(reinterpret_cast<char*>(&roi_search_distance_) -
    reinterpret_cast<char*>(&lane_range_)) + sizeof(roi_search_distance_));
  // @@protoc_insertion_point(copy_constructor:apollo.perception.lidar.MapManagerConfig)
}

void MapManagerConfig::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&lane_range_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&update_pose_) -
    reinterpret_cast<char*>(&lane_range_)) + sizeof(update_pose_));
roi_search_distance_ = 80;
}

MapManagerConfig::~MapManagerConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.lidar.MapManagerConfig)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void MapManagerConfig::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void MapManagerConfig::ArenaDtor(void* object) {
  MapManagerConfig* _this = reinterpret_cast< MapManagerConfig* >(object);
  (void)_this;
}
void MapManagerConfig::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void MapManagerConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void MapManagerConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.lidar.MapManagerConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    ::memset(&lane_range_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&update_pose_) -
        reinterpret_cast<char*>(&lane_range_)) + sizeof(update_pose_));
    roi_search_distance_ = 80;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* MapManagerConfig::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional bool update_pose = 1 [default = false];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          _Internal::set_has_update_pose(&has_bits);
          update_pose_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional double roi_search_distance = 2 [default = 80];
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 17)) {
          _Internal::set_has_roi_search_distance(&has_bits);
          roi_search_distance_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional double lane_range = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 25)) {
          _Internal::set_has_lane_range(&has_bits);
          lane_range_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional double max_depth = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 33)) {
          _Internal::set_has_max_depth(&has_bits);
          max_depth_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* MapManagerConfig::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.lidar.MapManagerConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional bool update_pose = 1 [default = false];
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(1, this->_internal_update_pose(), target);
  }

  // optional double roi_search_distance = 2 [default = 80];
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_roi_search_distance(), target);
  }

  // optional double lane_range = 3;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(3, this->_internal_lane_range(), target);
  }

  // optional double max_depth = 4;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(4, this->_internal_max_depth(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.lidar.MapManagerConfig)
  return target;
}

size_t MapManagerConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.lidar.MapManagerConfig)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    // optional double lane_range = 3;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 + 8;
    }

    // optional double max_depth = 4;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 + 8;
    }

    // optional bool update_pose = 1 [default = false];
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 + 1;
    }

    // optional double roi_search_distance = 2 [default = 80];
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 + 8;
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData MapManagerConfig::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    MapManagerConfig::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*MapManagerConfig::GetClassData() const { return &_class_data_; }

void MapManagerConfig::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<MapManagerConfig *>(to)->MergeFrom(
      static_cast<const MapManagerConfig &>(from));
}


void MapManagerConfig::MergeFrom(const MapManagerConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.lidar.MapManagerConfig)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      lane_range_ = from.lane_range_;
    }
    if (cached_has_bits & 0x00000002u) {
      max_depth_ = from.max_depth_;
    }
    if (cached_has_bits & 0x00000004u) {
      update_pose_ = from.update_pose_;
    }
    if (cached_has_bits & 0x00000008u) {
      roi_search_distance_ = from.roi_search_distance_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void MapManagerConfig::CopyFrom(const MapManagerConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.lidar.MapManagerConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MapManagerConfig::IsInitialized() const {
  return true;
}

void MapManagerConfig::InternalSwap(MapManagerConfig* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(MapManagerConfig, update_pose_)
      + sizeof(MapManagerConfig::update_pose_)
      - PROTOBUF_FIELD_OFFSET(MapManagerConfig, lane_range_)>(
          reinterpret_cast<char*>(&lane_range_),
          reinterpret_cast<char*>(&other->lane_range_));
  swap(roi_search_distance_, other->roi_search_distance_);
}

::PROTOBUF_NAMESPACE_ID::Metadata MapManagerConfig::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto_getter, &descriptor_table_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto_once,
      file_level_metadata_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::perception::lidar::MapManagerConfig* Arena::CreateMaybeMessage< ::apollo::perception::lidar::MapManagerConfig >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::perception::lidar::MapManagerConfig >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>