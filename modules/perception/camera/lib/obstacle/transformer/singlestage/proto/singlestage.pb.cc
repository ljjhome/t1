// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/camera/lib/obstacle/transformer/singlestage/proto/singlestage.proto

#include "modules/perception/camera/lib/obstacle/transformer/singlestage/proto/singlestage.pb.h"

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
namespace singlestage {
constexpr SinglestageParam::SinglestageParam(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : min_dimension_val_(0.2f)
  , check_dimension_(true){}
struct SinglestageParamDefaultTypeInternal {
  constexpr SinglestageParamDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~SinglestageParamDefaultTypeInternal() {}
  union {
    SinglestageParam _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT SinglestageParamDefaultTypeInternal _SinglestageParam_default_instance_;
}  // namespace singlestage
}  // namespace camera
}  // namespace perception
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::perception::camera::singlestage::SinglestageParam, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::camera::singlestage::SinglestageParam, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::perception::camera::singlestage::SinglestageParam, min_dimension_val_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::camera::singlestage::SinglestageParam, check_dimension_),
  0,
  1,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, -1, sizeof(::apollo::perception::camera::singlestage::SinglestageParam)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::perception::camera::singlestage::_SinglestageParam_default_instance_),
};

const char descriptor_table_protodef_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\nVmodules/perception/camera/lib/obstacle"
  "/transformer/singlestage/proto/singlesta"
  "ge.proto\022$apollo.perception.camera.singl"
  "estage\"Q\n\020SinglestageParam\022\036\n\021min_dimens"
  "ion_val\030\001 \001(\002:\0030.2\022\035\n\017check_dimension\030\002 "
  "\001(\010:\004true"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto = {
  false, false, 209, descriptor_table_protodef_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto, "modules/perception/camera/lib/obstacle/transformer/singlestage/proto/singlestage.proto", 
  &descriptor_table_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto::offsets,
  file_level_metadata_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto, file_level_enum_descriptors_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto, file_level_service_descriptors_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto_getter() {
  return &descriptor_table_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto(&descriptor_table_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto);
namespace apollo {
namespace perception {
namespace camera {
namespace singlestage {

// ===================================================================

class SinglestageParam::_Internal {
 public:
  using HasBits = decltype(std::declval<SinglestageParam>()._has_bits_);
  static void set_has_min_dimension_val(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_check_dimension(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

SinglestageParam::SinglestageParam(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.perception.camera.singlestage.SinglestageParam)
}
SinglestageParam::SinglestageParam(const SinglestageParam& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&min_dimension_val_, &from.min_dimension_val_,
    static_cast<size_t>(reinterpret_cast<char*>(&check_dimension_) -
    reinterpret_cast<char*>(&min_dimension_val_)) + sizeof(check_dimension_));
  // @@protoc_insertion_point(copy_constructor:apollo.perception.camera.singlestage.SinglestageParam)
}

void SinglestageParam::SharedCtor() {
min_dimension_val_ = 0.2f;
check_dimension_ = true;
}

SinglestageParam::~SinglestageParam() {
  // @@protoc_insertion_point(destructor:apollo.perception.camera.singlestage.SinglestageParam)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void SinglestageParam::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void SinglestageParam::ArenaDtor(void* object) {
  SinglestageParam* _this = reinterpret_cast< SinglestageParam* >(object);
  (void)_this;
}
void SinglestageParam::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void SinglestageParam::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void SinglestageParam::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.camera.singlestage.SinglestageParam)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    min_dimension_val_ = 0.2f;
    check_dimension_ = true;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* SinglestageParam::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional float min_dimension_val = 1 [default = 0.2];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 13)) {
          _Internal::set_has_min_dimension_val(&has_bits);
          min_dimension_val_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // optional bool check_dimension = 2 [default = true];
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          _Internal::set_has_check_dimension(&has_bits);
          check_dimension_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* SinglestageParam::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.camera.singlestage.SinglestageParam)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional float min_dimension_val = 1 [default = 0.2];
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(1, this->_internal_min_dimension_val(), target);
  }

  // optional bool check_dimension = 2 [default = true];
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(2, this->_internal_check_dimension(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.camera.singlestage.SinglestageParam)
  return target;
}

size_t SinglestageParam::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.camera.singlestage.SinglestageParam)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    // optional float min_dimension_val = 1 [default = 0.2];
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 + 4;
    }

    // optional bool check_dimension = 2 [default = true];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 + 1;
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData SinglestageParam::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    SinglestageParam::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*SinglestageParam::GetClassData() const { return &_class_data_; }

void SinglestageParam::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<SinglestageParam *>(to)->MergeFrom(
      static_cast<const SinglestageParam &>(from));
}


void SinglestageParam::MergeFrom(const SinglestageParam& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.camera.singlestage.SinglestageParam)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      min_dimension_val_ = from.min_dimension_val_;
    }
    if (cached_has_bits & 0x00000002u) {
      check_dimension_ = from.check_dimension_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void SinglestageParam::CopyFrom(const SinglestageParam& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.camera.singlestage.SinglestageParam)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SinglestageParam::IsInitialized() const {
  return true;
}

void SinglestageParam::InternalSwap(SinglestageParam* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(min_dimension_val_, other->min_dimension_val_);
  swap(check_dimension_, other->check_dimension_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SinglestageParam::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto_getter, &descriptor_table_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto_once,
      file_level_metadata_modules_2fperception_2fcamera_2flib_2fobstacle_2ftransformer_2fsinglestage_2fproto_2fsinglestage_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace singlestage
}  // namespace camera
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::perception::camera::singlestage::SinglestageParam* Arena::CreateMaybeMessage< ::apollo::perception::camera::singlestage::SinglestageParam >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::perception::camera::singlestage::SinglestageParam >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>