// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/proto/map_manager_config.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3018000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3018000 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto;
namespace apollo {
namespace perception {
namespace lidar {
class MapManagerConfig;
struct MapManagerConfigDefaultTypeInternal;
extern MapManagerConfigDefaultTypeInternal _MapManagerConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::perception::lidar::MapManagerConfig* Arena::CreateMaybeMessage<::apollo::perception::lidar::MapManagerConfig>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

class MapManagerConfig final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.perception.lidar.MapManagerConfig) */ {
 public:
  inline MapManagerConfig() : MapManagerConfig(nullptr) {}
  ~MapManagerConfig() override;
  explicit constexpr MapManagerConfig(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  MapManagerConfig(const MapManagerConfig& from);
  MapManagerConfig(MapManagerConfig&& from) noexcept
    : MapManagerConfig() {
    *this = ::std::move(from);
  }

  inline MapManagerConfig& operator=(const MapManagerConfig& from) {
    CopyFrom(from);
    return *this;
  }
  inline MapManagerConfig& operator=(MapManagerConfig&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const MapManagerConfig& default_instance() {
    return *internal_default_instance();
  }
  static inline const MapManagerConfig* internal_default_instance() {
    return reinterpret_cast<const MapManagerConfig*>(
               &_MapManagerConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(MapManagerConfig& a, MapManagerConfig& b) {
    a.Swap(&b);
  }
  inline void Swap(MapManagerConfig* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(MapManagerConfig* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MapManagerConfig* New() const final {
    return new MapManagerConfig();
  }

  MapManagerConfig* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MapManagerConfig>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const MapManagerConfig& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const MapManagerConfig& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to, const ::PROTOBUF_NAMESPACE_ID::Message& from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(MapManagerConfig* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.perception.lidar.MapManagerConfig";
  }
  protected:
  explicit MapManagerConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kLaneRangeFieldNumber = 3,
    kMaxDepthFieldNumber = 4,
    kUpdatePoseFieldNumber = 1,
    kRoiSearchDistanceFieldNumber = 2,
  };
  // optional double lane_range = 3;
  bool has_lane_range() const;
  private:
  bool _internal_has_lane_range() const;
  public:
  void clear_lane_range();
  double lane_range() const;
  void set_lane_range(double value);
  private:
  double _internal_lane_range() const;
  void _internal_set_lane_range(double value);
  public:

  // optional double max_depth = 4;
  bool has_max_depth() const;
  private:
  bool _internal_has_max_depth() const;
  public:
  void clear_max_depth();
  double max_depth() const;
  void set_max_depth(double value);
  private:
  double _internal_max_depth() const;
  void _internal_set_max_depth(double value);
  public:

  // optional bool update_pose = 1 [default = false];
  bool has_update_pose() const;
  private:
  bool _internal_has_update_pose() const;
  public:
  void clear_update_pose();
  bool update_pose() const;
  void set_update_pose(bool value);
  private:
  bool _internal_update_pose() const;
  void _internal_set_update_pose(bool value);
  public:

  // optional double roi_search_distance = 2 [default = 80];
  bool has_roi_search_distance() const;
  private:
  bool _internal_has_roi_search_distance() const;
  public:
  void clear_roi_search_distance();
  double roi_search_distance() const;
  void set_roi_search_distance(double value);
  private:
  double _internal_roi_search_distance() const;
  void _internal_set_roi_search_distance(double value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.perception.lidar.MapManagerConfig)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  double lane_range_;
  double max_depth_;
  bool update_pose_;
  double roi_search_distance_;
  friend struct ::TableStruct_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MapManagerConfig

// optional bool update_pose = 1 [default = false];
inline bool MapManagerConfig::_internal_has_update_pose() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool MapManagerConfig::has_update_pose() const {
  return _internal_has_update_pose();
}
inline void MapManagerConfig::clear_update_pose() {
  update_pose_ = false;
  _has_bits_[0] &= ~0x00000004u;
}
inline bool MapManagerConfig::_internal_update_pose() const {
  return update_pose_;
}
inline bool MapManagerConfig::update_pose() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.MapManagerConfig.update_pose)
  return _internal_update_pose();
}
inline void MapManagerConfig::_internal_set_update_pose(bool value) {
  _has_bits_[0] |= 0x00000004u;
  update_pose_ = value;
}
inline void MapManagerConfig::set_update_pose(bool value) {
  _internal_set_update_pose(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.MapManagerConfig.update_pose)
}

// optional double roi_search_distance = 2 [default = 80];
inline bool MapManagerConfig::_internal_has_roi_search_distance() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool MapManagerConfig::has_roi_search_distance() const {
  return _internal_has_roi_search_distance();
}
inline void MapManagerConfig::clear_roi_search_distance() {
  roi_search_distance_ = 80;
  _has_bits_[0] &= ~0x00000008u;
}
inline double MapManagerConfig::_internal_roi_search_distance() const {
  return roi_search_distance_;
}
inline double MapManagerConfig::roi_search_distance() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.MapManagerConfig.roi_search_distance)
  return _internal_roi_search_distance();
}
inline void MapManagerConfig::_internal_set_roi_search_distance(double value) {
  _has_bits_[0] |= 0x00000008u;
  roi_search_distance_ = value;
}
inline void MapManagerConfig::set_roi_search_distance(double value) {
  _internal_set_roi_search_distance(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.MapManagerConfig.roi_search_distance)
}

// optional double lane_range = 3;
inline bool MapManagerConfig::_internal_has_lane_range() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool MapManagerConfig::has_lane_range() const {
  return _internal_has_lane_range();
}
inline void MapManagerConfig::clear_lane_range() {
  lane_range_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline double MapManagerConfig::_internal_lane_range() const {
  return lane_range_;
}
inline double MapManagerConfig::lane_range() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.MapManagerConfig.lane_range)
  return _internal_lane_range();
}
inline void MapManagerConfig::_internal_set_lane_range(double value) {
  _has_bits_[0] |= 0x00000001u;
  lane_range_ = value;
}
inline void MapManagerConfig::set_lane_range(double value) {
  _internal_set_lane_range(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.MapManagerConfig.lane_range)
}

// optional double max_depth = 4;
inline bool MapManagerConfig::_internal_has_max_depth() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool MapManagerConfig::has_max_depth() const {
  return _internal_has_max_depth();
}
inline void MapManagerConfig::clear_max_depth() {
  max_depth_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline double MapManagerConfig::_internal_max_depth() const {
  return max_depth_;
}
inline double MapManagerConfig::max_depth() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.MapManagerConfig.max_depth)
  return _internal_max_depth();
}
inline void MapManagerConfig::_internal_set_max_depth(double value) {
  _has_bits_[0] |= 0x00000002u;
  max_depth_ = value;
}
inline void MapManagerConfig::set_max_depth(double value) {
  _internal_set_max_depth(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.MapManagerConfig.max_depth)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2fproto_2fmap_5fmanager_5fconfig_2eproto
