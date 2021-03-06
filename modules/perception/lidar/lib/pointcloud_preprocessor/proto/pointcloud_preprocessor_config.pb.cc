// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/lidar/lib/pointcloud_preprocessor/proto/pointcloud_preprocessor_config.proto

#include "modules/perception/lidar/lib/pointcloud_preprocessor/proto/pointcloud_preprocessor_config.pb.h"

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
constexpr PointCloudPreprocessorConfig::PointCloudPreprocessorConfig(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : box_forward_x_(0)
  , box_backward_x_(0)
  , filter_nearby_box_points_(false)
  , filter_high_z_points_(false)
  , box_forward_y_(0)
  , box_backward_y_(0)
  , filter_naninf_points_(true)
  , z_threshold_(5){}
struct PointCloudPreprocessorConfigDefaultTypeInternal {
  constexpr PointCloudPreprocessorConfigDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~PointCloudPreprocessorConfigDefaultTypeInternal() {}
  union {
    PointCloudPreprocessorConfig _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PointCloudPreprocessorConfigDefaultTypeInternal _PointCloudPreprocessorConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::PointCloudPreprocessorConfig, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::PointCloudPreprocessorConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::PointCloudPreprocessorConfig, filter_naninf_points_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::PointCloudPreprocessorConfig, filter_nearby_box_points_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::PointCloudPreprocessorConfig, box_forward_x_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::PointCloudPreprocessorConfig, box_backward_x_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::PointCloudPreprocessorConfig, box_forward_y_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::PointCloudPreprocessorConfig, box_backward_y_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::PointCloudPreprocessorConfig, filter_high_z_points_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::PointCloudPreprocessorConfig, z_threshold_),
  6,
  2,
  0,
  1,
  4,
  5,
  3,
  7,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 14, -1, sizeof(::apollo::perception::lidar::PointCloudPreprocessorConfig)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::perception::lidar::_PointCloudPreprocessorConfig_default_instance_),
};

const char descriptor_table_protodef_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n_modules/perception/lidar/lib/pointclou"
  "d_preprocessor/proto/pointcloud_preproce"
  "ssor_config.proto\022\027apollo.perception.lid"
  "ar\"\222\002\n\034PointCloudPreprocessorConfig\022\"\n\024f"
  "ilter_naninf_points\030\001 \001(\010:\004true\022\'\n\030filte"
  "r_nearby_box_points\030\002 \001(\010:\005false\022\030\n\rbox_"
  "forward_x\030\003 \001(\002:\0010\022\031\n\016box_backward_x\030\004 \001"
  "(\002:\0010\022\030\n\rbox_forward_y\030\005 \001(\002:\0010\022\031\n\016box_b"
  "ackward_y\030\006 \001(\002:\0010\022#\n\024filter_high_z_poin"
  "ts\030\007 \001(\010:\005false\022\026\n\013z_threshold\030\010 \001(\002:\0015"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto = {
  false, false, 399, descriptor_table_protodef_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto, "modules/perception/lidar/lib/pointcloud_preprocessor/proto/pointcloud_preprocessor_config.proto", 
  &descriptor_table_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto::offsets,
  file_level_metadata_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto, file_level_enum_descriptors_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto, file_level_service_descriptors_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto_getter() {
  return &descriptor_table_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto(&descriptor_table_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto);
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

class PointCloudPreprocessorConfig::_Internal {
 public:
  using HasBits = decltype(std::declval<PointCloudPreprocessorConfig>()._has_bits_);
  static void set_has_filter_naninf_points(HasBits* has_bits) {
    (*has_bits)[0] |= 64u;
  }
  static void set_has_filter_nearby_box_points(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_box_forward_x(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_box_backward_x(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_box_forward_y(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static void set_has_box_backward_y(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static void set_has_filter_high_z_points(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_z_threshold(HasBits* has_bits) {
    (*has_bits)[0] |= 128u;
  }
};

PointCloudPreprocessorConfig::PointCloudPreprocessorConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.perception.lidar.PointCloudPreprocessorConfig)
}
PointCloudPreprocessorConfig::PointCloudPreprocessorConfig(const PointCloudPreprocessorConfig& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&box_forward_x_, &from.box_forward_x_,
    static_cast<size_t>(reinterpret_cast<char*>(&z_threshold_) -
    reinterpret_cast<char*>(&box_forward_x_)) + sizeof(z_threshold_));
  // @@protoc_insertion_point(copy_constructor:apollo.perception.lidar.PointCloudPreprocessorConfig)
}

void PointCloudPreprocessorConfig::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&box_forward_x_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&box_backward_y_) -
    reinterpret_cast<char*>(&box_forward_x_)) + sizeof(box_backward_y_));
filter_naninf_points_ = true;
z_threshold_ = 5;
}

PointCloudPreprocessorConfig::~PointCloudPreprocessorConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.lidar.PointCloudPreprocessorConfig)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void PointCloudPreprocessorConfig::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void PointCloudPreprocessorConfig::ArenaDtor(void* object) {
  PointCloudPreprocessorConfig* _this = reinterpret_cast< PointCloudPreprocessorConfig* >(object);
  (void)_this;
}
void PointCloudPreprocessorConfig::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void PointCloudPreprocessorConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void PointCloudPreprocessorConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.lidar.PointCloudPreprocessorConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    ::memset(&box_forward_x_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&box_backward_y_) -
        reinterpret_cast<char*>(&box_forward_x_)) + sizeof(box_backward_y_));
    filter_naninf_points_ = true;
    z_threshold_ = 5;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* PointCloudPreprocessorConfig::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional bool filter_naninf_points = 1 [default = true];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          _Internal::set_has_filter_naninf_points(&has_bits);
          filter_naninf_points_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool filter_nearby_box_points = 2 [default = false];
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          _Internal::set_has_filter_nearby_box_points(&has_bits);
          filter_nearby_box_points_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional float box_forward_x = 3 [default = 0];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 29)) {
          _Internal::set_has_box_forward_x(&has_bits);
          box_forward_x_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // optional float box_backward_x = 4 [default = 0];
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 37)) {
          _Internal::set_has_box_backward_x(&has_bits);
          box_backward_x_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // optional float box_forward_y = 5 [default = 0];
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 45)) {
          _Internal::set_has_box_forward_y(&has_bits);
          box_forward_y_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // optional float box_backward_y = 6 [default = 0];
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 53)) {
          _Internal::set_has_box_backward_y(&has_bits);
          box_backward_y_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // optional bool filter_high_z_points = 7 [default = false];
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 56)) {
          _Internal::set_has_filter_high_z_points(&has_bits);
          filter_high_z_points_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional float z_threshold = 8 [default = 5];
      case 8:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 69)) {
          _Internal::set_has_z_threshold(&has_bits);
          z_threshold_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
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

::PROTOBUF_NAMESPACE_ID::uint8* PointCloudPreprocessorConfig::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.lidar.PointCloudPreprocessorConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional bool filter_naninf_points = 1 [default = true];
  if (cached_has_bits & 0x00000040u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(1, this->_internal_filter_naninf_points(), target);
  }

  // optional bool filter_nearby_box_points = 2 [default = false];
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(2, this->_internal_filter_nearby_box_points(), target);
  }

  // optional float box_forward_x = 3 [default = 0];
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(3, this->_internal_box_forward_x(), target);
  }

  // optional float box_backward_x = 4 [default = 0];
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(4, this->_internal_box_backward_x(), target);
  }

  // optional float box_forward_y = 5 [default = 0];
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(5, this->_internal_box_forward_y(), target);
  }

  // optional float box_backward_y = 6 [default = 0];
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(6, this->_internal_box_backward_y(), target);
  }

  // optional bool filter_high_z_points = 7 [default = false];
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(7, this->_internal_filter_high_z_points(), target);
  }

  // optional float z_threshold = 8 [default = 5];
  if (cached_has_bits & 0x00000080u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(8, this->_internal_z_threshold(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.lidar.PointCloudPreprocessorConfig)
  return target;
}

size_t PointCloudPreprocessorConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.lidar.PointCloudPreprocessorConfig)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    // optional float box_forward_x = 3 [default = 0];
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 + 4;
    }

    // optional float box_backward_x = 4 [default = 0];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 + 4;
    }

    // optional bool filter_nearby_box_points = 2 [default = false];
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 + 1;
    }

    // optional bool filter_high_z_points = 7 [default = false];
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 + 1;
    }

    // optional float box_forward_y = 5 [default = 0];
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 + 4;
    }

    // optional float box_backward_y = 6 [default = 0];
    if (cached_has_bits & 0x00000020u) {
      total_size += 1 + 4;
    }

    // optional bool filter_naninf_points = 1 [default = true];
    if (cached_has_bits & 0x00000040u) {
      total_size += 1 + 1;
    }

    // optional float z_threshold = 8 [default = 5];
    if (cached_has_bits & 0x00000080u) {
      total_size += 1 + 4;
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData PointCloudPreprocessorConfig::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    PointCloudPreprocessorConfig::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*PointCloudPreprocessorConfig::GetClassData() const { return &_class_data_; }

void PointCloudPreprocessorConfig::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<PointCloudPreprocessorConfig *>(to)->MergeFrom(
      static_cast<const PointCloudPreprocessorConfig &>(from));
}


void PointCloudPreprocessorConfig::MergeFrom(const PointCloudPreprocessorConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.lidar.PointCloudPreprocessorConfig)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    if (cached_has_bits & 0x00000001u) {
      box_forward_x_ = from.box_forward_x_;
    }
    if (cached_has_bits & 0x00000002u) {
      box_backward_x_ = from.box_backward_x_;
    }
    if (cached_has_bits & 0x00000004u) {
      filter_nearby_box_points_ = from.filter_nearby_box_points_;
    }
    if (cached_has_bits & 0x00000008u) {
      filter_high_z_points_ = from.filter_high_z_points_;
    }
    if (cached_has_bits & 0x00000010u) {
      box_forward_y_ = from.box_forward_y_;
    }
    if (cached_has_bits & 0x00000020u) {
      box_backward_y_ = from.box_backward_y_;
    }
    if (cached_has_bits & 0x00000040u) {
      filter_naninf_points_ = from.filter_naninf_points_;
    }
    if (cached_has_bits & 0x00000080u) {
      z_threshold_ = from.z_threshold_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void PointCloudPreprocessorConfig::CopyFrom(const PointCloudPreprocessorConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.lidar.PointCloudPreprocessorConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PointCloudPreprocessorConfig::IsInitialized() const {
  return true;
}

void PointCloudPreprocessorConfig::InternalSwap(PointCloudPreprocessorConfig* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(PointCloudPreprocessorConfig, box_backward_y_)
      + sizeof(PointCloudPreprocessorConfig::box_backward_y_)
      - PROTOBUF_FIELD_OFFSET(PointCloudPreprocessorConfig, box_forward_x_)>(
          reinterpret_cast<char*>(&box_forward_x_),
          reinterpret_cast<char*>(&other->box_forward_x_));
  swap(filter_naninf_points_, other->filter_naninf_points_);
  swap(z_threshold_, other->z_threshold_);
}

::PROTOBUF_NAMESPACE_ID::Metadata PointCloudPreprocessorConfig::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto_getter, &descriptor_table_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto_once,
      file_level_metadata_modules_2fperception_2flidar_2flib_2fpointcloud_5fpreprocessor_2fproto_2fpointcloud_5fpreprocessor_5fconfig_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::perception::lidar::PointCloudPreprocessorConfig* Arena::CreateMaybeMessage< ::apollo::perception::lidar::PointCloudPreprocessorConfig >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::perception::lidar::PointCloudPreprocessorConfig >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
