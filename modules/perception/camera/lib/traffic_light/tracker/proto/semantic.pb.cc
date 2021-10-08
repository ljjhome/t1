// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/camera/lib/traffic_light/tracker/proto/semantic.proto

#include "modules/perception/camera/lib/traffic_light/tracker/proto/semantic.pb.h"

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
namespace camera {
namespace traffic_light {
namespace tracker {
constexpr SemanticReviseParam::SemanticReviseParam(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : hysteretic_threshold_count_(2)
  , revise_time_second_(1.5f)
  , blink_threshold_second_(0.4f){}
struct SemanticReviseParamDefaultTypeInternal {
  constexpr SemanticReviseParamDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~SemanticReviseParamDefaultTypeInternal() {}
  union {
    SemanticReviseParam _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT SemanticReviseParamDefaultTypeInternal _SemanticReviseParam_default_instance_;
}  // namespace tracker
}  // namespace traffic_light
}  // namespace camera
}  // namespace perception
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::perception::camera::traffic_light::tracker::SemanticReviseParam, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::camera::traffic_light::tracker::SemanticReviseParam, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::perception::camera::traffic_light::tracker::SemanticReviseParam, revise_time_second_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::camera::traffic_light::tracker::SemanticReviseParam, blink_threshold_second_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::camera::traffic_light::tracker::SemanticReviseParam, hysteretic_threshold_count_),
  1,
  2,
  0,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, -1, sizeof(::apollo::perception::camera::traffic_light::tracker::SemanticReviseParam)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::perception::camera::traffic_light::tracker::_SemanticReviseParam_default_instance_),
};

const char descriptor_table_protodef_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\nHmodules/perception/camera/lib/traffic_"
  "light/tracker/proto/semantic.proto\022.apol"
  "lo.perception.camera.traffic_light.track"
  "er\"\202\001\n\023SemanticReviseParam\022\037\n\022revise_tim"
  "e_second\030\001 \001(\002:\0031.5\022#\n\026blink_threshold_s"
  "econd\030\002 \001(\002:\0030.4\022%\n\032hysteretic_threshold"
  "_count\030\003 \001(\005:\0012"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto = {
  false, false, 255, descriptor_table_protodef_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto, "modules/perception/camera/lib/traffic_light/tracker/proto/semantic.proto", 
  &descriptor_table_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto::offsets,
  file_level_metadata_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto, file_level_enum_descriptors_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto, file_level_service_descriptors_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto_getter() {
  return &descriptor_table_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto(&descriptor_table_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto);
namespace apollo {
namespace perception {
namespace camera {
namespace traffic_light {
namespace tracker {

// ===================================================================

class SemanticReviseParam::_Internal {
 public:
  using HasBits = decltype(std::declval<SemanticReviseParam>()._has_bits_);
  static void set_has_revise_time_second(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_blink_threshold_second(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_hysteretic_threshold_count(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

SemanticReviseParam::SemanticReviseParam(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.perception.camera.traffic_light.tracker.SemanticReviseParam)
}
SemanticReviseParam::SemanticReviseParam(const SemanticReviseParam& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&hysteretic_threshold_count_, &from.hysteretic_threshold_count_,
    static_cast<size_t>(reinterpret_cast<char*>(&blink_threshold_second_) -
    reinterpret_cast<char*>(&hysteretic_threshold_count_)) + sizeof(blink_threshold_second_));
  // @@protoc_insertion_point(copy_constructor:apollo.perception.camera.traffic_light.tracker.SemanticReviseParam)
}

void SemanticReviseParam::SharedCtor() {
hysteretic_threshold_count_ = 2;
revise_time_second_ = 1.5f;
blink_threshold_second_ = 0.4f;
}

SemanticReviseParam::~SemanticReviseParam() {
  // @@protoc_insertion_point(destructor:apollo.perception.camera.traffic_light.tracker.SemanticReviseParam)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void SemanticReviseParam::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void SemanticReviseParam::ArenaDtor(void* object) {
  SemanticReviseParam* _this = reinterpret_cast< SemanticReviseParam* >(object);
  (void)_this;
}
void SemanticReviseParam::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void SemanticReviseParam::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void SemanticReviseParam::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.camera.traffic_light.tracker.SemanticReviseParam)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    hysteretic_threshold_count_ = 2;
    revise_time_second_ = 1.5f;
    blink_threshold_second_ = 0.4f;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* SemanticReviseParam::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional float revise_time_second = 1 [default = 1.5];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 13)) {
          _Internal::set_has_revise_time_second(&has_bits);
          revise_time_second_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // optional float blink_threshold_second = 2 [default = 0.4];
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 21)) {
          _Internal::set_has_blink_threshold_second(&has_bits);
          blink_threshold_second_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // optional int32 hysteretic_threshold_count = 3 [default = 2];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          _Internal::set_has_hysteretic_threshold_count(&has_bits);
          hysteretic_threshold_count_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* SemanticReviseParam::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.camera.traffic_light.tracker.SemanticReviseParam)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional float revise_time_second = 1 [default = 1.5];
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(1, this->_internal_revise_time_second(), target);
  }

  // optional float blink_threshold_second = 2 [default = 0.4];
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(2, this->_internal_blink_threshold_second(), target);
  }

  // optional int32 hysteretic_threshold_count = 3 [default = 2];
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(3, this->_internal_hysteretic_threshold_count(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.camera.traffic_light.tracker.SemanticReviseParam)
  return target;
}

size_t SemanticReviseParam::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.camera.traffic_light.tracker.SemanticReviseParam)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional int32 hysteretic_threshold_count = 3 [default = 2];
    if (cached_has_bits & 0x00000001u) {
      total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32SizePlusOne(this->_internal_hysteretic_threshold_count());
    }

    // optional float revise_time_second = 1 [default = 1.5];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 + 4;
    }

    // optional float blink_threshold_second = 2 [default = 0.4];
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 + 4;
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData SemanticReviseParam::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    SemanticReviseParam::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*SemanticReviseParam::GetClassData() const { return &_class_data_; }

void SemanticReviseParam::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<SemanticReviseParam *>(to)->MergeFrom(
      static_cast<const SemanticReviseParam &>(from));
}


void SemanticReviseParam::MergeFrom(const SemanticReviseParam& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.camera.traffic_light.tracker.SemanticReviseParam)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      hysteretic_threshold_count_ = from.hysteretic_threshold_count_;
    }
    if (cached_has_bits & 0x00000002u) {
      revise_time_second_ = from.revise_time_second_;
    }
    if (cached_has_bits & 0x00000004u) {
      blink_threshold_second_ = from.blink_threshold_second_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void SemanticReviseParam::CopyFrom(const SemanticReviseParam& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.camera.traffic_light.tracker.SemanticReviseParam)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SemanticReviseParam::IsInitialized() const {
  return true;
}

void SemanticReviseParam::InternalSwap(SemanticReviseParam* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(hysteretic_threshold_count_, other->hysteretic_threshold_count_);
  swap(revise_time_second_, other->revise_time_second_);
  swap(blink_threshold_second_, other->blink_threshold_second_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SemanticReviseParam::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto_getter, &descriptor_table_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto_once,
      file_level_metadata_modules_2fperception_2fcamera_2flib_2ftraffic_5flight_2ftracker_2fproto_2fsemantic_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace tracker
}  // namespace traffic_light
}  // namespace camera
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::perception::camera::traffic_light::tracker::SemanticReviseParam* Arena::CreateMaybeMessage< ::apollo::perception::camera::traffic_light::tracker::SemanticReviseParam >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::perception::camera::traffic_light::tracker::SemanticReviseParam >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>