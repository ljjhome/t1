// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/lidar/lib/ground_detector/ground_service_detector/proto/ground_service_detector_config.proto

#include "modules/perception/lidar/lib/ground_detector/ground_service_detector/proto/ground_service_detector_config.pb.h"

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
constexpr GroundServiceDetectorConfig::GroundServiceDetectorConfig(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : ground_threshold_(0.25){}
struct GroundServiceDetectorConfigDefaultTypeInternal {
  constexpr GroundServiceDetectorConfigDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~GroundServiceDetectorConfigDefaultTypeInternal() {}
  union {
    GroundServiceDetectorConfig _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT GroundServiceDetectorConfigDefaultTypeInternal _GroundServiceDetectorConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::GroundServiceDetectorConfig, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::GroundServiceDetectorConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::GroundServiceDetectorConfig, ground_threshold_),
  0,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, -1, sizeof(::apollo::perception::lidar::GroundServiceDetectorConfig)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::perception::lidar::_GroundServiceDetectorConfig_default_instance_),
};

const char descriptor_table_protodef_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\nomodules/perception/lidar/lib/ground_de"
  "tector/ground_service_detector/proto/gro"
  "und_service_detector_config.proto\022\027apoll"
  "o.perception.lidar\"=\n\033GroundServiceDetec"
  "torConfig\022\036\n\020ground_threshold\030\001 \001(\001:\0040.2"
  "5"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto = {
  false, false, 201, descriptor_table_protodef_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto, "modules/perception/lidar/lib/ground_detector/ground_service_detector/proto/ground_service_detector_config.proto", 
  &descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto::offsets,
  file_level_metadata_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto, file_level_enum_descriptors_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto, file_level_service_descriptors_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto_getter() {
  return &descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto(&descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto);
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

class GroundServiceDetectorConfig::_Internal {
 public:
  using HasBits = decltype(std::declval<GroundServiceDetectorConfig>()._has_bits_);
  static void set_has_ground_threshold(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

GroundServiceDetectorConfig::GroundServiceDetectorConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.perception.lidar.GroundServiceDetectorConfig)
}
GroundServiceDetectorConfig::GroundServiceDetectorConfig(const GroundServiceDetectorConfig& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ground_threshold_ = from.ground_threshold_;
  // @@protoc_insertion_point(copy_constructor:apollo.perception.lidar.GroundServiceDetectorConfig)
}

void GroundServiceDetectorConfig::SharedCtor() {
ground_threshold_ = 0.25;
}

GroundServiceDetectorConfig::~GroundServiceDetectorConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.lidar.GroundServiceDetectorConfig)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void GroundServiceDetectorConfig::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void GroundServiceDetectorConfig::ArenaDtor(void* object) {
  GroundServiceDetectorConfig* _this = reinterpret_cast< GroundServiceDetectorConfig* >(object);
  (void)_this;
}
void GroundServiceDetectorConfig::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void GroundServiceDetectorConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void GroundServiceDetectorConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.lidar.GroundServiceDetectorConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ground_threshold_ = 0.25;
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* GroundServiceDetectorConfig::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional double ground_threshold = 1 [default = 0.25];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          _Internal::set_has_ground_threshold(&has_bits);
          ground_threshold_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* GroundServiceDetectorConfig::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.lidar.GroundServiceDetectorConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double ground_threshold = 1 [default = 0.25];
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_ground_threshold(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.lidar.GroundServiceDetectorConfig)
  return target;
}

size_t GroundServiceDetectorConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.lidar.GroundServiceDetectorConfig)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // optional double ground_threshold = 1 [default = 0.25];
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 1 + 8;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData GroundServiceDetectorConfig::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    GroundServiceDetectorConfig::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GroundServiceDetectorConfig::GetClassData() const { return &_class_data_; }

void GroundServiceDetectorConfig::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<GroundServiceDetectorConfig *>(to)->MergeFrom(
      static_cast<const GroundServiceDetectorConfig &>(from));
}


void GroundServiceDetectorConfig::MergeFrom(const GroundServiceDetectorConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.lidar.GroundServiceDetectorConfig)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_ground_threshold()) {
    _internal_set_ground_threshold(from._internal_ground_threshold());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void GroundServiceDetectorConfig::CopyFrom(const GroundServiceDetectorConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.lidar.GroundServiceDetectorConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool GroundServiceDetectorConfig::IsInitialized() const {
  return true;
}

void GroundServiceDetectorConfig::InternalSwap(GroundServiceDetectorConfig* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(ground_threshold_, other->ground_threshold_);
}

::PROTOBUF_NAMESPACE_ID::Metadata GroundServiceDetectorConfig::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto_getter, &descriptor_table_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto_once,
      file_level_metadata_modules_2fperception_2flidar_2flib_2fground_5fdetector_2fground_5fservice_5fdetector_2fproto_2fground_5fservice_5fdetector_5fconfig_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::perception::lidar::GroundServiceDetectorConfig* Arena::CreateMaybeMessage< ::apollo::perception::lidar::GroundServiceDetectorConfig >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::perception::lidar::GroundServiceDetectorConfig >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
