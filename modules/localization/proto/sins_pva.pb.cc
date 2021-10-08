// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/localization/proto/sins_pva.proto

#include "modules/localization/proto/sins_pva.pb.h"

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
namespace localization {
constexpr IntegSinsPva::IntegSinsPva(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : pva_covar_()
  , header_(nullptr)
  , position_(nullptr)
  , velocity_(nullptr)
  , attitude_(nullptr)
  , init_and_alignment_(false){}
struct IntegSinsPvaDefaultTypeInternal {
  constexpr IntegSinsPvaDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~IntegSinsPvaDefaultTypeInternal() {}
  union {
    IntegSinsPva _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT IntegSinsPvaDefaultTypeInternal _IntegSinsPva_default_instance_;
}  // namespace localization
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2flocalization_2fproto_2fsins_5fpva_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2flocalization_2fproto_2fsins_5fpva_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2flocalization_2fproto_2fsins_5fpva_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2flocalization_2fproto_2fsins_5fpva_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::localization::IntegSinsPva, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::localization::IntegSinsPva, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::localization::IntegSinsPva, header_),
  PROTOBUF_FIELD_OFFSET(::apollo::localization::IntegSinsPva, position_),
  PROTOBUF_FIELD_OFFSET(::apollo::localization::IntegSinsPva, velocity_),
  PROTOBUF_FIELD_OFFSET(::apollo::localization::IntegSinsPva, attitude_),
  PROTOBUF_FIELD_OFFSET(::apollo::localization::IntegSinsPva, pva_covar_),
  PROTOBUF_FIELD_OFFSET(::apollo::localization::IntegSinsPva, init_and_alignment_),
  0,
  1,
  2,
  3,
  ~0u,
  4,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 12, -1, sizeof(::apollo::localization::IntegSinsPva)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::localization::_IntegSinsPva_default_instance_),
};

const char descriptor_table_protodef_modules_2flocalization_2fproto_2fsins_5fpva_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n)modules/localization/proto/sins_pva.pr"
  "oto\022\023apollo.localization\032!modules/common"
  "/proto/header.proto\032#modules/common/prot"
  "o/geometry.proto\"\347\001\n\014IntegSinsPva\022%\n\006hea"
  "der\030\001 \001(\0132\025.apollo.common.Header\022)\n\010posi"
  "tion\030\002 \001(\0132\027.apollo.common.PointLLH\022(\n\010v"
  "elocity\030\003 \001(\0132\026.apollo.common.Point3D\022(\n"
  "\010attitude\030\004 \001(\0132\026.apollo.common.Point3D\022"
  "\025\n\tpva_covar\030\005 \003(\001B\002\020\001\022\032\n\022init_and_align"
  "ment\030\006 \001(\010"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2flocalization_2fproto_2fsins_5fpva_2eproto_deps[2] = {
  &::descriptor_table_modules_2fcommon_2fproto_2fgeometry_2eproto,
  &::descriptor_table_modules_2fcommon_2fproto_2fheader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2flocalization_2fproto_2fsins_5fpva_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2flocalization_2fproto_2fsins_5fpva_2eproto = {
  false, false, 370, descriptor_table_protodef_modules_2flocalization_2fproto_2fsins_5fpva_2eproto, "modules/localization/proto/sins_pva.proto", 
  &descriptor_table_modules_2flocalization_2fproto_2fsins_5fpva_2eproto_once, descriptor_table_modules_2flocalization_2fproto_2fsins_5fpva_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_modules_2flocalization_2fproto_2fsins_5fpva_2eproto::offsets,
  file_level_metadata_modules_2flocalization_2fproto_2fsins_5fpva_2eproto, file_level_enum_descriptors_modules_2flocalization_2fproto_2fsins_5fpva_2eproto, file_level_service_descriptors_modules_2flocalization_2fproto_2fsins_5fpva_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2flocalization_2fproto_2fsins_5fpva_2eproto_getter() {
  return &descriptor_table_modules_2flocalization_2fproto_2fsins_5fpva_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2flocalization_2fproto_2fsins_5fpva_2eproto(&descriptor_table_modules_2flocalization_2fproto_2fsins_5fpva_2eproto);
namespace apollo {
namespace localization {

// ===================================================================

class IntegSinsPva::_Internal {
 public:
  using HasBits = decltype(std::declval<IntegSinsPva>()._has_bits_);
  static const ::apollo::common::Header& header(const IntegSinsPva* msg);
  static void set_has_header(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::apollo::common::PointLLH& position(const IntegSinsPva* msg);
  static void set_has_position(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static const ::apollo::common::Point3D& velocity(const IntegSinsPva* msg);
  static void set_has_velocity(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static const ::apollo::common::Point3D& attitude(const IntegSinsPva* msg);
  static void set_has_attitude(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_init_and_alignment(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
};

const ::apollo::common::Header&
IntegSinsPva::_Internal::header(const IntegSinsPva* msg) {
  return *msg->header_;
}
const ::apollo::common::PointLLH&
IntegSinsPva::_Internal::position(const IntegSinsPva* msg) {
  return *msg->position_;
}
const ::apollo::common::Point3D&
IntegSinsPva::_Internal::velocity(const IntegSinsPva* msg) {
  return *msg->velocity_;
}
const ::apollo::common::Point3D&
IntegSinsPva::_Internal::attitude(const IntegSinsPva* msg) {
  return *msg->attitude_;
}
void IntegSinsPva::clear_header() {
  if (header_ != nullptr) header_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void IntegSinsPva::clear_position() {
  if (position_ != nullptr) position_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
void IntegSinsPva::clear_velocity() {
  if (velocity_ != nullptr) velocity_->Clear();
  _has_bits_[0] &= ~0x00000004u;
}
void IntegSinsPva::clear_attitude() {
  if (attitude_ != nullptr) attitude_->Clear();
  _has_bits_[0] &= ~0x00000008u;
}
IntegSinsPva::IntegSinsPva(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  pva_covar_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.localization.IntegSinsPva)
}
IntegSinsPva::IntegSinsPva(const IntegSinsPva& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      pva_covar_(from.pva_covar_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::apollo::common::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  if (from._internal_has_position()) {
    position_ = new ::apollo::common::PointLLH(*from.position_);
  } else {
    position_ = nullptr;
  }
  if (from._internal_has_velocity()) {
    velocity_ = new ::apollo::common::Point3D(*from.velocity_);
  } else {
    velocity_ = nullptr;
  }
  if (from._internal_has_attitude()) {
    attitude_ = new ::apollo::common::Point3D(*from.attitude_);
  } else {
    attitude_ = nullptr;
  }
  init_and_alignment_ = from.init_and_alignment_;
  // @@protoc_insertion_point(copy_constructor:apollo.localization.IntegSinsPva)
}

void IntegSinsPva::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&init_and_alignment_) -
    reinterpret_cast<char*>(&header_)) + sizeof(init_and_alignment_));
}

IntegSinsPva::~IntegSinsPva() {
  // @@protoc_insertion_point(destructor:apollo.localization.IntegSinsPva)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void IntegSinsPva::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete position_;
  if (this != internal_default_instance()) delete velocity_;
  if (this != internal_default_instance()) delete attitude_;
}

void IntegSinsPva::ArenaDtor(void* object) {
  IntegSinsPva* _this = reinterpret_cast< IntegSinsPva* >(object);
  (void)_this;
}
void IntegSinsPva::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void IntegSinsPva::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void IntegSinsPva::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.localization.IntegSinsPva)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  pva_covar_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(header_ != nullptr);
      header_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(position_ != nullptr);
      position_->Clear();
    }
    if (cached_has_bits & 0x00000004u) {
      GOOGLE_DCHECK(velocity_ != nullptr);
      velocity_->Clear();
    }
    if (cached_has_bits & 0x00000008u) {
      GOOGLE_DCHECK(attitude_ != nullptr);
      attitude_->Clear();
    }
  }
  init_and_alignment_ = false;
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* IntegSinsPva::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // optional .apollo.common.PointLLH position = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_position(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional .apollo.common.Point3D velocity = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr = ctx->ParseMessage(_internal_mutable_velocity(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional .apollo.common.Point3D attitude = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 34)) {
          ptr = ctx->ParseMessage(_internal_mutable_attitude(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated double pva_covar = 5 [packed = true];
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 42)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedDoubleParser(_internal_mutable_pva_covar(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 41) {
          _internal_add_pva_covar(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr));
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional bool init_and_alignment = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 48)) {
          _Internal::set_has_init_and_alignment(&has_bits);
          init_and_alignment_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* IntegSinsPva::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.localization.IntegSinsPva)
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

  // optional .apollo.common.PointLLH position = 2;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::position(this), target, stream);
  }

  // optional .apollo.common.Point3D velocity = 3;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        3, _Internal::velocity(this), target, stream);
  }

  // optional .apollo.common.Point3D attitude = 4;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        4, _Internal::attitude(this), target, stream);
  }

  // repeated double pva_covar = 5 [packed = true];
  if (this->_internal_pva_covar_size() > 0) {
    target = stream->WriteFixedPacked(5, _internal_pva_covar(), target);
  }

  // optional bool init_and_alignment = 6;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(6, this->_internal_init_and_alignment(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.localization.IntegSinsPva)
  return target;
}

size_t IntegSinsPva::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.localization.IntegSinsPva)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated double pva_covar = 5 [packed = true];
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_pva_covar_size());
    size_t data_size = 8UL * count;
    if (data_size > 0) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
            static_cast<::PROTOBUF_NAMESPACE_ID::int32>(data_size));
    }
    total_size += data_size;
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    // optional .apollo.common.Header header = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *header_);
    }

    // optional .apollo.common.PointLLH position = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *position_);
    }

    // optional .apollo.common.Point3D velocity = 3;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *velocity_);
    }

    // optional .apollo.common.Point3D attitude = 4;
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *attitude_);
    }

    // optional bool init_and_alignment = 6;
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 + 1;
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData IntegSinsPva::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    IntegSinsPva::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*IntegSinsPva::GetClassData() const { return &_class_data_; }

void IntegSinsPva::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<IntegSinsPva *>(to)->MergeFrom(
      static_cast<const IntegSinsPva &>(from));
}


void IntegSinsPva::MergeFrom(const IntegSinsPva& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.localization.IntegSinsPva)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  pva_covar_.MergeFrom(from.pva_covar_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_header()->::apollo::common::Header::MergeFrom(from._internal_header());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_position()->::apollo::common::PointLLH::MergeFrom(from._internal_position());
    }
    if (cached_has_bits & 0x00000004u) {
      _internal_mutable_velocity()->::apollo::common::Point3D::MergeFrom(from._internal_velocity());
    }
    if (cached_has_bits & 0x00000008u) {
      _internal_mutable_attitude()->::apollo::common::Point3D::MergeFrom(from._internal_attitude());
    }
    if (cached_has_bits & 0x00000010u) {
      init_and_alignment_ = from.init_and_alignment_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void IntegSinsPva::CopyFrom(const IntegSinsPva& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.localization.IntegSinsPva)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool IntegSinsPva::IsInitialized() const {
  return true;
}

void IntegSinsPva::InternalSwap(IntegSinsPva* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  pva_covar_.InternalSwap(&other->pva_covar_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(IntegSinsPva, init_and_alignment_)
      + sizeof(IntegSinsPva::init_and_alignment_)
      - PROTOBUF_FIELD_OFFSET(IntegSinsPva, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata IntegSinsPva::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2flocalization_2fproto_2fsins_5fpva_2eproto_getter, &descriptor_table_modules_2flocalization_2fproto_2fsins_5fpva_2eproto_once,
      file_level_metadata_modules_2flocalization_2fproto_2fsins_5fpva_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace localization
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::localization::IntegSinsPva* Arena::CreateMaybeMessage< ::apollo::localization::IntegSinsPva >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::localization::IntegSinsPva >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
