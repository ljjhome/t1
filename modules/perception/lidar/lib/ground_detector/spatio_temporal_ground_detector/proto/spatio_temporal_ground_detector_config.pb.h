// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector/proto/spatio_temporal_ground_detector_config.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto;
namespace apollo {
namespace perception {
namespace lidar {
class SpatioTemporalGroundDetectorConfig;
struct SpatioTemporalGroundDetectorConfigDefaultTypeInternal;
extern SpatioTemporalGroundDetectorConfigDefaultTypeInternal _SpatioTemporalGroundDetectorConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::perception::lidar::SpatioTemporalGroundDetectorConfig* Arena::CreateMaybeMessage<::apollo::perception::lidar::SpatioTemporalGroundDetectorConfig>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

class SpatioTemporalGroundDetectorConfig final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig) */ {
 public:
  inline SpatioTemporalGroundDetectorConfig() : SpatioTemporalGroundDetectorConfig(nullptr) {}
  ~SpatioTemporalGroundDetectorConfig() override;
  explicit constexpr SpatioTemporalGroundDetectorConfig(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  SpatioTemporalGroundDetectorConfig(const SpatioTemporalGroundDetectorConfig& from);
  SpatioTemporalGroundDetectorConfig(SpatioTemporalGroundDetectorConfig&& from) noexcept
    : SpatioTemporalGroundDetectorConfig() {
    *this = ::std::move(from);
  }

  inline SpatioTemporalGroundDetectorConfig& operator=(const SpatioTemporalGroundDetectorConfig& from) {
    CopyFrom(from);
    return *this;
  }
  inline SpatioTemporalGroundDetectorConfig& operator=(SpatioTemporalGroundDetectorConfig&& from) noexcept {
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
  static const SpatioTemporalGroundDetectorConfig& default_instance() {
    return *internal_default_instance();
  }
  static inline const SpatioTemporalGroundDetectorConfig* internal_default_instance() {
    return reinterpret_cast<const SpatioTemporalGroundDetectorConfig*>(
               &_SpatioTemporalGroundDetectorConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(SpatioTemporalGroundDetectorConfig& a, SpatioTemporalGroundDetectorConfig& b) {
    a.Swap(&b);
  }
  inline void Swap(SpatioTemporalGroundDetectorConfig* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(SpatioTemporalGroundDetectorConfig* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline SpatioTemporalGroundDetectorConfig* New() const final {
    return new SpatioTemporalGroundDetectorConfig();
  }

  SpatioTemporalGroundDetectorConfig* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<SpatioTemporalGroundDetectorConfig>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const SpatioTemporalGroundDetectorConfig& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const SpatioTemporalGroundDetectorConfig& from);
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
  void InternalSwap(SpatioTemporalGroundDetectorConfig* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.perception.lidar.SpatioTemporalGroundDetectorConfig";
  }
  protected:
  explicit SpatioTemporalGroundDetectorConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kUseRoiFieldNumber = 7,
    kUseGroundServiceFieldNumber = 8,
    kGridSizeFieldNumber = 1,
    kGroundThresFieldNumber = 2,
    kRoiRadXFieldNumber = 3,
    kRoiRadYFieldNumber = 4,
    kRoiRadZFieldNumber = 5,
    kNrSmoothIterFieldNumber = 6,
  };
  // optional bool use_roi = 7 [default = true];
  bool has_use_roi() const;
  private:
  bool _internal_has_use_roi() const;
  public:
  void clear_use_roi();
  bool use_roi() const;
  void set_use_roi(bool value);
  private:
  bool _internal_use_roi() const;
  void _internal_set_use_roi(bool value);
  public:

  // optional bool use_ground_service = 8 [default = true];
  bool has_use_ground_service() const;
  private:
  bool _internal_has_use_ground_service() const;
  public:
  void clear_use_ground_service();
  bool use_ground_service() const;
  void set_use_ground_service(bool value);
  private:
  bool _internal_use_ground_service() const;
  void _internal_set_use_ground_service(bool value);
  public:

  // optional uint32 grid_size = 1 [default = 16];
  bool has_grid_size() const;
  private:
  bool _internal_has_grid_size() const;
  public:
  void clear_grid_size();
  ::PROTOBUF_NAMESPACE_ID::uint32 grid_size() const;
  void set_grid_size(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_grid_size() const;
  void _internal_set_grid_size(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional float ground_thres = 2 [default = 0.25];
  bool has_ground_thres() const;
  private:
  bool _internal_has_ground_thres() const;
  public:
  void clear_ground_thres();
  float ground_thres() const;
  void set_ground_thres(float value);
  private:
  float _internal_ground_thres() const;
  void _internal_set_ground_thres(float value);
  public:

  // optional float roi_rad_x = 3 [default = 120];
  bool has_roi_rad_x() const;
  private:
  bool _internal_has_roi_rad_x() const;
  public:
  void clear_roi_rad_x();
  float roi_rad_x() const;
  void set_roi_rad_x(float value);
  private:
  float _internal_roi_rad_x() const;
  void _internal_set_roi_rad_x(float value);
  public:

  // optional float roi_rad_y = 4 [default = 120];
  bool has_roi_rad_y() const;
  private:
  bool _internal_has_roi_rad_y() const;
  public:
  void clear_roi_rad_y();
  float roi_rad_y() const;
  void set_roi_rad_y(float value);
  private:
  float _internal_roi_rad_y() const;
  void _internal_set_roi_rad_y(float value);
  public:

  // optional float roi_rad_z = 5 [default = 100];
  bool has_roi_rad_z() const;
  private:
  bool _internal_has_roi_rad_z() const;
  public:
  void clear_roi_rad_z();
  float roi_rad_z() const;
  void set_roi_rad_z(float value);
  private:
  float _internal_roi_rad_z() const;
  void _internal_set_roi_rad_z(float value);
  public:

  // optional uint32 nr_smooth_iter = 6 [default = 5];
  bool has_nr_smooth_iter() const;
  private:
  bool _internal_has_nr_smooth_iter() const;
  public:
  void clear_nr_smooth_iter();
  ::PROTOBUF_NAMESPACE_ID::uint32 nr_smooth_iter() const;
  void set_nr_smooth_iter(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_nr_smooth_iter() const;
  void _internal_set_nr_smooth_iter(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  bool use_roi_;
  bool use_ground_service_;
  ::PROTOBUF_NAMESPACE_ID::uint32 grid_size_;
  float ground_thres_;
  float roi_rad_x_;
  float roi_rad_y_;
  float roi_rad_z_;
  ::PROTOBUF_NAMESPACE_ID::uint32 nr_smooth_iter_;
  friend struct ::TableStruct_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SpatioTemporalGroundDetectorConfig

// optional uint32 grid_size = 1 [default = 16];
inline bool SpatioTemporalGroundDetectorConfig::_internal_has_grid_size() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool SpatioTemporalGroundDetectorConfig::has_grid_size() const {
  return _internal_has_grid_size();
}
inline void SpatioTemporalGroundDetectorConfig::clear_grid_size() {
  grid_size_ = 16u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 SpatioTemporalGroundDetectorConfig::_internal_grid_size() const {
  return grid_size_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 SpatioTemporalGroundDetectorConfig::grid_size() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig.grid_size)
  return _internal_grid_size();
}
inline void SpatioTemporalGroundDetectorConfig::_internal_set_grid_size(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  grid_size_ = value;
}
inline void SpatioTemporalGroundDetectorConfig::set_grid_size(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_grid_size(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig.grid_size)
}

// optional float ground_thres = 2 [default = 0.25];
inline bool SpatioTemporalGroundDetectorConfig::_internal_has_ground_thres() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool SpatioTemporalGroundDetectorConfig::has_ground_thres() const {
  return _internal_has_ground_thres();
}
inline void SpatioTemporalGroundDetectorConfig::clear_ground_thres() {
  ground_thres_ = 0.25f;
  _has_bits_[0] &= ~0x00000008u;
}
inline float SpatioTemporalGroundDetectorConfig::_internal_ground_thres() const {
  return ground_thres_;
}
inline float SpatioTemporalGroundDetectorConfig::ground_thres() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig.ground_thres)
  return _internal_ground_thres();
}
inline void SpatioTemporalGroundDetectorConfig::_internal_set_ground_thres(float value) {
  _has_bits_[0] |= 0x00000008u;
  ground_thres_ = value;
}
inline void SpatioTemporalGroundDetectorConfig::set_ground_thres(float value) {
  _internal_set_ground_thres(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig.ground_thres)
}

// optional float roi_rad_x = 3 [default = 120];
inline bool SpatioTemporalGroundDetectorConfig::_internal_has_roi_rad_x() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool SpatioTemporalGroundDetectorConfig::has_roi_rad_x() const {
  return _internal_has_roi_rad_x();
}
inline void SpatioTemporalGroundDetectorConfig::clear_roi_rad_x() {
  roi_rad_x_ = 120;
  _has_bits_[0] &= ~0x00000010u;
}
inline float SpatioTemporalGroundDetectorConfig::_internal_roi_rad_x() const {
  return roi_rad_x_;
}
inline float SpatioTemporalGroundDetectorConfig::roi_rad_x() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig.roi_rad_x)
  return _internal_roi_rad_x();
}
inline void SpatioTemporalGroundDetectorConfig::_internal_set_roi_rad_x(float value) {
  _has_bits_[0] |= 0x00000010u;
  roi_rad_x_ = value;
}
inline void SpatioTemporalGroundDetectorConfig::set_roi_rad_x(float value) {
  _internal_set_roi_rad_x(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig.roi_rad_x)
}

// optional float roi_rad_y = 4 [default = 120];
inline bool SpatioTemporalGroundDetectorConfig::_internal_has_roi_rad_y() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool SpatioTemporalGroundDetectorConfig::has_roi_rad_y() const {
  return _internal_has_roi_rad_y();
}
inline void SpatioTemporalGroundDetectorConfig::clear_roi_rad_y() {
  roi_rad_y_ = 120;
  _has_bits_[0] &= ~0x00000020u;
}
inline float SpatioTemporalGroundDetectorConfig::_internal_roi_rad_y() const {
  return roi_rad_y_;
}
inline float SpatioTemporalGroundDetectorConfig::roi_rad_y() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig.roi_rad_y)
  return _internal_roi_rad_y();
}
inline void SpatioTemporalGroundDetectorConfig::_internal_set_roi_rad_y(float value) {
  _has_bits_[0] |= 0x00000020u;
  roi_rad_y_ = value;
}
inline void SpatioTemporalGroundDetectorConfig::set_roi_rad_y(float value) {
  _internal_set_roi_rad_y(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig.roi_rad_y)
}

// optional float roi_rad_z = 5 [default = 100];
inline bool SpatioTemporalGroundDetectorConfig::_internal_has_roi_rad_z() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool SpatioTemporalGroundDetectorConfig::has_roi_rad_z() const {
  return _internal_has_roi_rad_z();
}
inline void SpatioTemporalGroundDetectorConfig::clear_roi_rad_z() {
  roi_rad_z_ = 100;
  _has_bits_[0] &= ~0x00000040u;
}
inline float SpatioTemporalGroundDetectorConfig::_internal_roi_rad_z() const {
  return roi_rad_z_;
}
inline float SpatioTemporalGroundDetectorConfig::roi_rad_z() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig.roi_rad_z)
  return _internal_roi_rad_z();
}
inline void SpatioTemporalGroundDetectorConfig::_internal_set_roi_rad_z(float value) {
  _has_bits_[0] |= 0x00000040u;
  roi_rad_z_ = value;
}
inline void SpatioTemporalGroundDetectorConfig::set_roi_rad_z(float value) {
  _internal_set_roi_rad_z(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig.roi_rad_z)
}

// optional uint32 nr_smooth_iter = 6 [default = 5];
inline bool SpatioTemporalGroundDetectorConfig::_internal_has_nr_smooth_iter() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool SpatioTemporalGroundDetectorConfig::has_nr_smooth_iter() const {
  return _internal_has_nr_smooth_iter();
}
inline void SpatioTemporalGroundDetectorConfig::clear_nr_smooth_iter() {
  nr_smooth_iter_ = 5u;
  _has_bits_[0] &= ~0x00000080u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 SpatioTemporalGroundDetectorConfig::_internal_nr_smooth_iter() const {
  return nr_smooth_iter_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 SpatioTemporalGroundDetectorConfig::nr_smooth_iter() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig.nr_smooth_iter)
  return _internal_nr_smooth_iter();
}
inline void SpatioTemporalGroundDetectorConfig::_internal_set_nr_smooth_iter(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000080u;
  nr_smooth_iter_ = value;
}
inline void SpatioTemporalGroundDetectorConfig::set_nr_smooth_iter(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_nr_smooth_iter(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig.nr_smooth_iter)
}

// optional bool use_roi = 7 [default = true];
inline bool SpatioTemporalGroundDetectorConfig::_internal_has_use_roi() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool SpatioTemporalGroundDetectorConfig::has_use_roi() const {
  return _internal_has_use_roi();
}
inline void SpatioTemporalGroundDetectorConfig::clear_use_roi() {
  use_roi_ = true;
  _has_bits_[0] &= ~0x00000001u;
}
inline bool SpatioTemporalGroundDetectorConfig::_internal_use_roi() const {
  return use_roi_;
}
inline bool SpatioTemporalGroundDetectorConfig::use_roi() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig.use_roi)
  return _internal_use_roi();
}
inline void SpatioTemporalGroundDetectorConfig::_internal_set_use_roi(bool value) {
  _has_bits_[0] |= 0x00000001u;
  use_roi_ = value;
}
inline void SpatioTemporalGroundDetectorConfig::set_use_roi(bool value) {
  _internal_set_use_roi(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig.use_roi)
}

// optional bool use_ground_service = 8 [default = true];
inline bool SpatioTemporalGroundDetectorConfig::_internal_has_use_ground_service() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool SpatioTemporalGroundDetectorConfig::has_use_ground_service() const {
  return _internal_has_use_ground_service();
}
inline void SpatioTemporalGroundDetectorConfig::clear_use_ground_service() {
  use_ground_service_ = true;
  _has_bits_[0] &= ~0x00000002u;
}
inline bool SpatioTemporalGroundDetectorConfig::_internal_use_ground_service() const {
  return use_ground_service_;
}
inline bool SpatioTemporalGroundDetectorConfig::use_ground_service() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig.use_ground_service)
  return _internal_use_ground_service();
}
inline void SpatioTemporalGroundDetectorConfig::_internal_set_use_ground_service(bool value) {
  _has_bits_[0] |= 0x00000002u;
  use_ground_service_ = value;
}
inline void SpatioTemporalGroundDetectorConfig::set_use_ground_service(bool value) {
  _internal_set_use_ground_service(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig.use_ground_service)
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
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto
