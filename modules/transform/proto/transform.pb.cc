// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/transform/proto/transform.proto

#include "modules/transform/proto/transform.pb.h"

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
namespace transform {
constexpr Transform::Transform(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : translation_(nullptr)
  , rotation_(nullptr){}
struct TransformDefaultTypeInternal {
  constexpr TransformDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~TransformDefaultTypeInternal() {}
  union {
    Transform _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT TransformDefaultTypeInternal _Transform_default_instance_;
constexpr TransformStamped::TransformStamped(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : child_frame_id_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , header_(nullptr)
  , transform_(nullptr){}
struct TransformStampedDefaultTypeInternal {
  constexpr TransformStampedDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~TransformStampedDefaultTypeInternal() {}
  union {
    TransformStamped _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT TransformStampedDefaultTypeInternal _TransformStamped_default_instance_;
constexpr TransformStampeds::TransformStampeds(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : transforms_()
  , header_(nullptr){}
struct TransformStampedsDefaultTypeInternal {
  constexpr TransformStampedsDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~TransformStampedsDefaultTypeInternal() {}
  union {
    TransformStampeds _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT TransformStampedsDefaultTypeInternal _TransformStampeds_default_instance_;
}  // namespace transform
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2ftransform_2fproto_2ftransform_2eproto[3];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2ftransform_2fproto_2ftransform_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2ftransform_2fproto_2ftransform_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2ftransform_2fproto_2ftransform_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::transform::Transform, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::transform::Transform, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::transform::Transform, translation_),
  PROTOBUF_FIELD_OFFSET(::apollo::transform::Transform, rotation_),
  0,
  1,
  PROTOBUF_FIELD_OFFSET(::apollo::transform::TransformStamped, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::transform::TransformStamped, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::transform::TransformStamped, header_),
  PROTOBUF_FIELD_OFFSET(::apollo::transform::TransformStamped, child_frame_id_),
  PROTOBUF_FIELD_OFFSET(::apollo::transform::TransformStamped, transform_),
  1,
  0,
  2,
  PROTOBUF_FIELD_OFFSET(::apollo::transform::TransformStampeds, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::transform::TransformStampeds, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::transform::TransformStampeds, header_),
  PROTOBUF_FIELD_OFFSET(::apollo::transform::TransformStampeds, transforms_),
  0,
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, -1, sizeof(::apollo::transform::Transform)},
  { 10, 19, -1, sizeof(::apollo::transform::TransformStamped)},
  { 22, 30, -1, sizeof(::apollo::transform::TransformStampeds)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::transform::_Transform_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::transform::_TransformStamped_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::transform::_TransformStampeds_default_instance_),
};

const char descriptor_table_protodef_modules_2ftransform_2fproto_2ftransform_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\'modules/transform/proto/transform.prot"
  "o\022\020apollo.transform\032!modules/common/prot"
  "o/header.proto\032#modules/common/proto/geo"
  "metry.proto\"e\n\tTransform\022+\n\013translation\030"
  "\001 \001(\0132\026.apollo.common.Point3D\022+\n\010rotatio"
  "n\030\002 \001(\0132\031.apollo.common.Quaternion\"\201\001\n\020T"
  "ransformStamped\022%\n\006header\030\001 \001(\0132\025.apollo"
  ".common.Header\022\026\n\016child_frame_id\030\002 \001(\t\022."
  "\n\ttransform\030\003 \001(\0132\033.apollo.transform.Tra"
  "nsform\"r\n\021TransformStampeds\022%\n\006header\030\001 "
  "\001(\0132\025.apollo.common.Header\0226\n\ntransforms"
  "\030\002 \003(\0132\".apollo.transform.TransformStamp"
  "ed"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto_deps[2] = {
  &::descriptor_table_modules_2fcommon_2fproto_2fgeometry_2eproto,
  &::descriptor_table_modules_2fcommon_2fproto_2fheader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto = {
  false, false, 482, descriptor_table_protodef_modules_2ftransform_2fproto_2ftransform_2eproto, "modules/transform/proto/transform.proto", 
  &descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto_once, descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto_deps, 2, 3,
  schemas, file_default_instances, TableStruct_modules_2ftransform_2fproto_2ftransform_2eproto::offsets,
  file_level_metadata_modules_2ftransform_2fproto_2ftransform_2eproto, file_level_enum_descriptors_modules_2ftransform_2fproto_2ftransform_2eproto, file_level_service_descriptors_modules_2ftransform_2fproto_2ftransform_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto_getter() {
  return &descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2ftransform_2fproto_2ftransform_2eproto(&descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto);
namespace apollo {
namespace transform {

// ===================================================================

class Transform::_Internal {
 public:
  using HasBits = decltype(std::declval<Transform>()._has_bits_);
  static const ::apollo::common::Point3D& translation(const Transform* msg);
  static void set_has_translation(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::apollo::common::Quaternion& rotation(const Transform* msg);
  static void set_has_rotation(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

const ::apollo::common::Point3D&
Transform::_Internal::translation(const Transform* msg) {
  return *msg->translation_;
}
const ::apollo::common::Quaternion&
Transform::_Internal::rotation(const Transform* msg) {
  return *msg->rotation_;
}
void Transform::clear_translation() {
  if (translation_ != nullptr) translation_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void Transform::clear_rotation() {
  if (rotation_ != nullptr) rotation_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
Transform::Transform(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.transform.Transform)
}
Transform::Transform(const Transform& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_translation()) {
    translation_ = new ::apollo::common::Point3D(*from.translation_);
  } else {
    translation_ = nullptr;
  }
  if (from._internal_has_rotation()) {
    rotation_ = new ::apollo::common::Quaternion(*from.rotation_);
  } else {
    rotation_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.transform.Transform)
}

void Transform::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&translation_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&rotation_) -
    reinterpret_cast<char*>(&translation_)) + sizeof(rotation_));
}

Transform::~Transform() {
  // @@protoc_insertion_point(destructor:apollo.transform.Transform)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Transform::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete translation_;
  if (this != internal_default_instance()) delete rotation_;
}

void Transform::ArenaDtor(void* object) {
  Transform* _this = reinterpret_cast< Transform* >(object);
  (void)_this;
}
void Transform::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Transform::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Transform::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.transform.Transform)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(translation_ != nullptr);
      translation_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(rotation_ != nullptr);
      rotation_->Clear();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Transform::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional .apollo.common.Point3D translation = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_translation(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional .apollo.common.Quaternion rotation = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_rotation(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* Transform::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.transform.Transform)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.Point3D translation = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::translation(this), target, stream);
  }

  // optional .apollo.common.Quaternion rotation = 2;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::rotation(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.transform.Transform)
  return target;
}

size_t Transform::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.transform.Transform)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    // optional .apollo.common.Point3D translation = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *translation_);
    }

    // optional .apollo.common.Quaternion rotation = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *rotation_);
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Transform::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Transform::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Transform::GetClassData() const { return &_class_data_; }

void Transform::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Transform *>(to)->MergeFrom(
      static_cast<const Transform &>(from));
}


void Transform::MergeFrom(const Transform& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.transform.Transform)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_translation()->::apollo::common::Point3D::MergeFrom(from._internal_translation());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_rotation()->::apollo::common::Quaternion::MergeFrom(from._internal_rotation());
    }
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Transform::CopyFrom(const Transform& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.transform.Transform)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Transform::IsInitialized() const {
  return true;
}

void Transform::InternalSwap(Transform* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Transform, rotation_)
      + sizeof(Transform::rotation_)
      - PROTOBUF_FIELD_OFFSET(Transform, translation_)>(
          reinterpret_cast<char*>(&translation_),
          reinterpret_cast<char*>(&other->translation_));
}

::PROTOBUF_NAMESPACE_ID::Metadata Transform::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto_getter, &descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto_once,
      file_level_metadata_modules_2ftransform_2fproto_2ftransform_2eproto[0]);
}

// ===================================================================

class TransformStamped::_Internal {
 public:
  using HasBits = decltype(std::declval<TransformStamped>()._has_bits_);
  static const ::apollo::common::Header& header(const TransformStamped* msg);
  static void set_has_header(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_child_frame_id(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::apollo::transform::Transform& transform(const TransformStamped* msg);
  static void set_has_transform(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::apollo::common::Header&
TransformStamped::_Internal::header(const TransformStamped* msg) {
  return *msg->header_;
}
const ::apollo::transform::Transform&
TransformStamped::_Internal::transform(const TransformStamped* msg) {
  return *msg->transform_;
}
void TransformStamped::clear_header() {
  if (header_ != nullptr) header_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
TransformStamped::TransformStamped(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.transform.TransformStamped)
}
TransformStamped::TransformStamped(const TransformStamped& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  child_frame_id_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_child_frame_id()) {
    child_frame_id_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_child_frame_id(), 
      GetArenaForAllocation());
  }
  if (from._internal_has_header()) {
    header_ = new ::apollo::common::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  if (from._internal_has_transform()) {
    transform_ = new ::apollo::transform::Transform(*from.transform_);
  } else {
    transform_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.transform.TransformStamped)
}

void TransformStamped::SharedCtor() {
child_frame_id_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&transform_) -
    reinterpret_cast<char*>(&header_)) + sizeof(transform_));
}

TransformStamped::~TransformStamped() {
  // @@protoc_insertion_point(destructor:apollo.transform.TransformStamped)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void TransformStamped::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  child_frame_id_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete transform_;
}

void TransformStamped::ArenaDtor(void* object) {
  TransformStamped* _this = reinterpret_cast< TransformStamped* >(object);
  (void)_this;
}
void TransformStamped::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void TransformStamped::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void TransformStamped::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.transform.TransformStamped)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      child_frame_id_.ClearNonDefaultToEmpty();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(header_ != nullptr);
      header_->Clear();
    }
    if (cached_has_bits & 0x00000004u) {
      GOOGLE_DCHECK(transform_ != nullptr);
      transform_->Clear();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* TransformStamped::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional .apollo.common.Header header = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_header(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional string child_frame_id = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          auto str = _internal_mutable_child_frame_id();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.transform.TransformStamped.child_frame_id");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional .apollo.transform.Transform transform = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr = ctx->ParseMessage(_internal_mutable_transform(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* TransformStamped::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.transform.TransformStamped)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.Header header = 1;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // optional string child_frame_id = 2;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_child_frame_id().data(), static_cast<int>(this->_internal_child_frame_id().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.transform.TransformStamped.child_frame_id");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_child_frame_id(), target);
  }

  // optional .apollo.transform.Transform transform = 3;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        3, _Internal::transform(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.transform.TransformStamped)
  return target;
}

size_t TransformStamped::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.transform.TransformStamped)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional string child_frame_id = 2;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_child_frame_id());
    }

    // optional .apollo.common.Header header = 1;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *header_);
    }

    // optional .apollo.transform.Transform transform = 3;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *transform_);
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData TransformStamped::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    TransformStamped::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*TransformStamped::GetClassData() const { return &_class_data_; }

void TransformStamped::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<TransformStamped *>(to)->MergeFrom(
      static_cast<const TransformStamped &>(from));
}


void TransformStamped::MergeFrom(const TransformStamped& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.transform.TransformStamped)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_set_child_frame_id(from._internal_child_frame_id());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_header()->::apollo::common::Header::MergeFrom(from._internal_header());
    }
    if (cached_has_bits & 0x00000004u) {
      _internal_mutable_transform()->::apollo::transform::Transform::MergeFrom(from._internal_transform());
    }
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void TransformStamped::CopyFrom(const TransformStamped& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.transform.TransformStamped)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool TransformStamped::IsInitialized() const {
  return true;
}

void TransformStamped::InternalSwap(TransformStamped* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &child_frame_id_, lhs_arena,
      &other->child_frame_id_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(TransformStamped, transform_)
      + sizeof(TransformStamped::transform_)
      - PROTOBUF_FIELD_OFFSET(TransformStamped, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata TransformStamped::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto_getter, &descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto_once,
      file_level_metadata_modules_2ftransform_2fproto_2ftransform_2eproto[1]);
}

// ===================================================================

class TransformStampeds::_Internal {
 public:
  using HasBits = decltype(std::declval<TransformStampeds>()._has_bits_);
  static const ::apollo::common::Header& header(const TransformStampeds* msg);
  static void set_has_header(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

const ::apollo::common::Header&
TransformStampeds::_Internal::header(const TransformStampeds* msg) {
  return *msg->header_;
}
void TransformStampeds::clear_header() {
  if (header_ != nullptr) header_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
TransformStampeds::TransformStampeds(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  transforms_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.transform.TransformStampeds)
}
TransformStampeds::TransformStampeds(const TransformStampeds& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      transforms_(from.transforms_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::apollo::common::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.transform.TransformStampeds)
}

void TransformStampeds::SharedCtor() {
header_ = nullptr;
}

TransformStampeds::~TransformStampeds() {
  // @@protoc_insertion_point(destructor:apollo.transform.TransformStampeds)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void TransformStampeds::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete header_;
}

void TransformStampeds::ArenaDtor(void* object) {
  TransformStampeds* _this = reinterpret_cast< TransformStampeds* >(object);
  (void)_this;
}
void TransformStampeds::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void TransformStampeds::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void TransformStampeds::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.transform.TransformStampeds)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  transforms_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(header_ != nullptr);
    header_->Clear();
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* TransformStampeds::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional .apollo.common.Header header = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_header(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated .apollo.transform.TransformStamped transforms = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_transforms(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<18>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* TransformStampeds::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.transform.TransformStampeds)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.Header header = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // repeated .apollo.transform.TransformStamped transforms = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_transforms_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, this->_internal_transforms(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.transform.TransformStampeds)
  return target;
}

size_t TransformStampeds::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.transform.TransformStampeds)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .apollo.transform.TransformStamped transforms = 2;
  total_size += 1UL * this->_internal_transforms_size();
  for (const auto& msg : this->transforms_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // optional .apollo.common.Header header = 1;
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *header_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData TransformStampeds::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    TransformStampeds::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*TransformStampeds::GetClassData() const { return &_class_data_; }

void TransformStampeds::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<TransformStampeds *>(to)->MergeFrom(
      static_cast<const TransformStampeds &>(from));
}


void TransformStampeds::MergeFrom(const TransformStampeds& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.transform.TransformStampeds)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  transforms_.MergeFrom(from.transforms_);
  if (from._internal_has_header()) {
    _internal_mutable_header()->::apollo::common::Header::MergeFrom(from._internal_header());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void TransformStampeds::CopyFrom(const TransformStampeds& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.transform.TransformStampeds)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool TransformStampeds::IsInitialized() const {
  return true;
}

void TransformStampeds::InternalSwap(TransformStampeds* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  transforms_.InternalSwap(&other->transforms_);
  swap(header_, other->header_);
}

::PROTOBUF_NAMESPACE_ID::Metadata TransformStampeds::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto_getter, &descriptor_table_modules_2ftransform_2fproto_2ftransform_2eproto_once,
      file_level_metadata_modules_2ftransform_2fproto_2ftransform_2eproto[2]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace transform
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::transform::Transform* Arena::CreateMaybeMessage< ::apollo::transform::Transform >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::transform::Transform >(arena);
}
template<> PROTOBUF_NOINLINE ::apollo::transform::TransformStamped* Arena::CreateMaybeMessage< ::apollo::transform::TransformStamped >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::transform::TransformStamped >(arena);
}
template<> PROTOBUF_NOINLINE ::apollo::transform::TransformStampeds* Arena::CreateMaybeMessage< ::apollo::transform::TransformStampeds >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::transform::TransformStampeds >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
