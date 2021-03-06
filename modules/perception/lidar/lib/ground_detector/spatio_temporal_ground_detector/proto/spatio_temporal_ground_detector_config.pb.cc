// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector/proto/spatio_temporal_ground_detector_config.proto

#include "modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector/proto/spatio_temporal_ground_detector_config.pb.h"

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
constexpr SpatioTemporalGroundDetectorConfig::SpatioTemporalGroundDetectorConfig(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : use_roi_(true)
  , use_ground_service_(true)
  , grid_size_(16u)
  , ground_thres_(0.25f)
  , roi_rad_x_(120)
  , roi_rad_y_(120)
  , roi_rad_z_(100)
  , nr_smooth_iter_(5u){}
struct SpatioTemporalGroundDetectorConfigDefaultTypeInternal {
  constexpr SpatioTemporalGroundDetectorConfigDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~SpatioTemporalGroundDetectorConfigDefaultTypeInternal() {}
  union {
    SpatioTemporalGroundDetectorConfig _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT SpatioTemporalGroundDetectorConfigDefaultTypeInternal _SpatioTemporalGroundDetectorConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::SpatioTemporalGroundDetectorConfig, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::SpatioTemporalGroundDetectorConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::SpatioTemporalGroundDetectorConfig, grid_size_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::SpatioTemporalGroundDetectorConfig, ground_thres_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::SpatioTemporalGroundDetectorConfig, roi_rad_x_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::SpatioTemporalGroundDetectorConfig, roi_rad_y_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::SpatioTemporalGroundDetectorConfig, roi_rad_z_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::SpatioTemporalGroundDetectorConfig, nr_smooth_iter_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::SpatioTemporalGroundDetectorConfig, use_roi_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::SpatioTemporalGroundDetectorConfig, use_ground_service_),
  2,
  3,
  4,
  5,
  6,
  7,
  0,
  1,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 14, -1, sizeof(::apollo::perception::lidar::SpatioTemporalGroundDetectorConfig)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::perception::lidar::_SpatioTemporalGroundDetectorConfig_default_instance_),
};

const char descriptor_table_protodef_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\177modules/perception/lidar/lib/ground_de"
  "tector/spatio_temporal_ground_detector/p"
  "roto/spatio_temporal_ground_detector_con"
  "fig.proto\022\027apollo.perception.lidar\"\363\001\n\"S"
  "patioTemporalGroundDetectorConfig\022\025\n\tgri"
  "d_size\030\001 \001(\r:\00216\022\032\n\014ground_thres\030\002 \001(\002:\004"
  "0.25\022\026\n\troi_rad_x\030\003 \001(\002:\003120\022\026\n\troi_rad_"
  "y\030\004 \001(\002:\003120\022\026\n\troi_rad_z\030\005 \001(\002:\003100\022\031\n\016"
  "nr_smooth_iter\030\006 \001(\r:\0015\022\025\n\007use_roi\030\007 \001(\010"
  ":\004true\022 \n\022use_ground_service\030\010 \001(\010:\004true"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto = {
  false, false, 400, descriptor_table_protodef_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto, "modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector/proto/spatio_temporal_ground_detector_config.proto", 
  &descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto::offsets,
  file_level_metadata_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto, file_level_enum_descriptors_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto, file_level_service_descriptors_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto_getter() {
  return &descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto(&descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto);
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

class SpatioTemporalGroundDetectorConfig::_Internal {
 public:
  using HasBits = decltype(std::declval<SpatioTemporalGroundDetectorConfig>()._has_bits_);
  static void set_has_grid_size(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_ground_thres(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_roi_rad_x(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static void set_has_roi_rad_y(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static void set_has_roi_rad_z(HasBits* has_bits) {
    (*has_bits)[0] |= 64u;
  }
  static void set_has_nr_smooth_iter(HasBits* has_bits) {
    (*has_bits)[0] |= 128u;
  }
  static void set_has_use_roi(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_use_ground_service(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

SpatioTemporalGroundDetectorConfig::SpatioTemporalGroundDetectorConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig)
}
SpatioTemporalGroundDetectorConfig::SpatioTemporalGroundDetectorConfig(const SpatioTemporalGroundDetectorConfig& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&use_roi_, &from.use_roi_,
    static_cast<size_t>(reinterpret_cast<char*>(&nr_smooth_iter_) -
    reinterpret_cast<char*>(&use_roi_)) + sizeof(nr_smooth_iter_));
  // @@protoc_insertion_point(copy_constructor:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig)
}

void SpatioTemporalGroundDetectorConfig::SharedCtor() {
use_roi_ = true;
use_ground_service_ = true;
grid_size_ = 16u;
ground_thres_ = 0.25f;
roi_rad_x_ = 120;
roi_rad_y_ = 120;
roi_rad_z_ = 100;
nr_smooth_iter_ = 5u;
}

SpatioTemporalGroundDetectorConfig::~SpatioTemporalGroundDetectorConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void SpatioTemporalGroundDetectorConfig::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void SpatioTemporalGroundDetectorConfig::ArenaDtor(void* object) {
  SpatioTemporalGroundDetectorConfig* _this = reinterpret_cast< SpatioTemporalGroundDetectorConfig* >(object);
  (void)_this;
}
void SpatioTemporalGroundDetectorConfig::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void SpatioTemporalGroundDetectorConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void SpatioTemporalGroundDetectorConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    use_roi_ = true;
    use_ground_service_ = true;
    grid_size_ = 16u;
    ground_thres_ = 0.25f;
    roi_rad_x_ = 120;
    roi_rad_y_ = 120;
    roi_rad_z_ = 100;
    nr_smooth_iter_ = 5u;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* SpatioTemporalGroundDetectorConfig::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional uint32 grid_size = 1 [default = 16];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          _Internal::set_has_grid_size(&has_bits);
          grid_size_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional float ground_thres = 2 [default = 0.25];
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 21)) {
          _Internal::set_has_ground_thres(&has_bits);
          ground_thres_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // optional float roi_rad_x = 3 [default = 120];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 29)) {
          _Internal::set_has_roi_rad_x(&has_bits);
          roi_rad_x_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // optional float roi_rad_y = 4 [default = 120];
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 37)) {
          _Internal::set_has_roi_rad_y(&has_bits);
          roi_rad_y_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // optional float roi_rad_z = 5 [default = 100];
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 45)) {
          _Internal::set_has_roi_rad_z(&has_bits);
          roi_rad_z_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // optional uint32 nr_smooth_iter = 6 [default = 5];
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 48)) {
          _Internal::set_has_nr_smooth_iter(&has_bits);
          nr_smooth_iter_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool use_roi = 7 [default = true];
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 56)) {
          _Internal::set_has_use_roi(&has_bits);
          use_roi_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool use_ground_service = 8 [default = true];
      case 8:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 64)) {
          _Internal::set_has_use_ground_service(&has_bits);
          use_ground_service_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* SpatioTemporalGroundDetectorConfig::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional uint32 grid_size = 1 [default = 16];
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(1, this->_internal_grid_size(), target);
  }

  // optional float ground_thres = 2 [default = 0.25];
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(2, this->_internal_ground_thres(), target);
  }

  // optional float roi_rad_x = 3 [default = 120];
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(3, this->_internal_roi_rad_x(), target);
  }

  // optional float roi_rad_y = 4 [default = 120];
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(4, this->_internal_roi_rad_y(), target);
  }

  // optional float roi_rad_z = 5 [default = 100];
  if (cached_has_bits & 0x00000040u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(5, this->_internal_roi_rad_z(), target);
  }

  // optional uint32 nr_smooth_iter = 6 [default = 5];
  if (cached_has_bits & 0x00000080u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(6, this->_internal_nr_smooth_iter(), target);
  }

  // optional bool use_roi = 7 [default = true];
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(7, this->_internal_use_roi(), target);
  }

  // optional bool use_ground_service = 8 [default = true];
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(8, this->_internal_use_ground_service(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig)
  return target;
}

size_t SpatioTemporalGroundDetectorConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    // optional bool use_roi = 7 [default = true];
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 + 1;
    }

    // optional bool use_ground_service = 8 [default = true];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 + 1;
    }

    // optional uint32 grid_size = 1 [default = 16];
    if (cached_has_bits & 0x00000004u) {
      total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32SizePlusOne(this->_internal_grid_size());
    }

    // optional float ground_thres = 2 [default = 0.25];
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 + 4;
    }

    // optional float roi_rad_x = 3 [default = 120];
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 + 4;
    }

    // optional float roi_rad_y = 4 [default = 120];
    if (cached_has_bits & 0x00000020u) {
      total_size += 1 + 4;
    }

    // optional float roi_rad_z = 5 [default = 100];
    if (cached_has_bits & 0x00000040u) {
      total_size += 1 + 4;
    }

    // optional uint32 nr_smooth_iter = 6 [default = 5];
    if (cached_has_bits & 0x00000080u) {
      total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32SizePlusOne(this->_internal_nr_smooth_iter());
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData SpatioTemporalGroundDetectorConfig::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    SpatioTemporalGroundDetectorConfig::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*SpatioTemporalGroundDetectorConfig::GetClassData() const { return &_class_data_; }

void SpatioTemporalGroundDetectorConfig::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<SpatioTemporalGroundDetectorConfig *>(to)->MergeFrom(
      static_cast<const SpatioTemporalGroundDetectorConfig &>(from));
}


void SpatioTemporalGroundDetectorConfig::MergeFrom(const SpatioTemporalGroundDetectorConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    if (cached_has_bits & 0x00000001u) {
      use_roi_ = from.use_roi_;
    }
    if (cached_has_bits & 0x00000002u) {
      use_ground_service_ = from.use_ground_service_;
    }
    if (cached_has_bits & 0x00000004u) {
      grid_size_ = from.grid_size_;
    }
    if (cached_has_bits & 0x00000008u) {
      ground_thres_ = from.ground_thres_;
    }
    if (cached_has_bits & 0x00000010u) {
      roi_rad_x_ = from.roi_rad_x_;
    }
    if (cached_has_bits & 0x00000020u) {
      roi_rad_y_ = from.roi_rad_y_;
    }
    if (cached_has_bits & 0x00000040u) {
      roi_rad_z_ = from.roi_rad_z_;
    }
    if (cached_has_bits & 0x00000080u) {
      nr_smooth_iter_ = from.nr_smooth_iter_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void SpatioTemporalGroundDetectorConfig::CopyFrom(const SpatioTemporalGroundDetectorConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.lidar.SpatioTemporalGroundDetectorConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SpatioTemporalGroundDetectorConfig::IsInitialized() const {
  return true;
}

void SpatioTemporalGroundDetectorConfig::InternalSwap(SpatioTemporalGroundDetectorConfig* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(use_roi_, other->use_roi_);
  swap(use_ground_service_, other->use_ground_service_);
  swap(grid_size_, other->grid_size_);
  swap(ground_thres_, other->ground_thres_);
  swap(roi_rad_x_, other->roi_rad_x_);
  swap(roi_rad_y_, other->roi_rad_y_);
  swap(roi_rad_z_, other->roi_rad_z_);
  swap(nr_smooth_iter_, other->nr_smooth_iter_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SpatioTemporalGroundDetectorConfig::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto_getter, &descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto_once,
      file_level_metadata_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fspatio_5ftemporal_5fground_5fdetector_2fproto_2fspatio_5ftemporal_5fground_5fdetector_5fconfig_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::perception::lidar::SpatioTemporalGroundDetectorConfig* Arena::CreateMaybeMessage< ::apollo::perception::lidar::SpatioTemporalGroundDetectorConfig >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::perception::lidar::SpatioTemporalGroundDetectorConfig >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
