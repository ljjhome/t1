// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/map/proto/map_speed_bump.proto

#include "modules/map/proto/map_speed_bump.pb.h"

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
namespace hdmap {
constexpr SpeedBump::SpeedBump(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : overlap_id_()
  , position_()
  , id_(nullptr){}
struct SpeedBumpDefaultTypeInternal {
  constexpr SpeedBumpDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~SpeedBumpDefaultTypeInternal() {}
  union {
    SpeedBump _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT SpeedBumpDefaultTypeInternal _SpeedBump_default_instance_;
}  // namespace hdmap
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::SpeedBump, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::SpeedBump, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::SpeedBump, id_),
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::SpeedBump, overlap_id_),
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::SpeedBump, position_),
  0,
  ~0u,
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, -1, sizeof(::apollo::hdmap::SpeedBump)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::hdmap::_SpeedBump_default_instance_),
};

const char descriptor_table_protodef_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n&modules/map/proto/map_speed_bump.proto"
  "\022\014apollo.hdmap\032\036modules/map/proto/map_id"
  ".proto\032$modules/map/proto/map_geometry.p"
  "roto\"v\n\tSpeedBump\022\034\n\002id\030\001 \001(\0132\020.apollo.h"
  "dmap.Id\022$\n\noverlap_id\030\002 \003(\0132\020.apollo.hdm"
  "ap.Id\022%\n\010position\030\003 \003(\0132\023.apollo.hdmap.C"
  "urve"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto_deps[2] = {
  &::descriptor_table_modules_2fmap_2fproto_2fmap_5fgeometry_2eproto,
  &::descriptor_table_modules_2fmap_2fproto_2fmap_5fid_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto = {
  false, false, 244, descriptor_table_protodef_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto, "modules/map/proto/map_speed_bump.proto", 
  &descriptor_table_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto_once, descriptor_table_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto::offsets,
  file_level_metadata_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto, file_level_enum_descriptors_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto, file_level_service_descriptors_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto_getter() {
  return &descriptor_table_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto(&descriptor_table_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto);
namespace apollo {
namespace hdmap {

// ===================================================================

class SpeedBump::_Internal {
 public:
  using HasBits = decltype(std::declval<SpeedBump>()._has_bits_);
  static const ::apollo::hdmap::Id& id(const SpeedBump* msg);
  static void set_has_id(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

const ::apollo::hdmap::Id&
SpeedBump::_Internal::id(const SpeedBump* msg) {
  return *msg->id_;
}
void SpeedBump::clear_id() {
  if (id_ != nullptr) id_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void SpeedBump::clear_overlap_id() {
  overlap_id_.Clear();
}
void SpeedBump::clear_position() {
  position_.Clear();
}
SpeedBump::SpeedBump(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  overlap_id_(arena),
  position_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.hdmap.SpeedBump)
}
SpeedBump::SpeedBump(const SpeedBump& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      overlap_id_(from.overlap_id_),
      position_(from.position_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_id()) {
    id_ = new ::apollo::hdmap::Id(*from.id_);
  } else {
    id_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.hdmap.SpeedBump)
}

void SpeedBump::SharedCtor() {
id_ = nullptr;
}

SpeedBump::~SpeedBump() {
  // @@protoc_insertion_point(destructor:apollo.hdmap.SpeedBump)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void SpeedBump::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete id_;
}

void SpeedBump::ArenaDtor(void* object) {
  SpeedBump* _this = reinterpret_cast< SpeedBump* >(object);
  (void)_this;
}
void SpeedBump::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void SpeedBump::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void SpeedBump::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.hdmap.SpeedBump)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  overlap_id_.Clear();
  position_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(id_ != nullptr);
    id_->Clear();
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* SpeedBump::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional .apollo.hdmap.Id id = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_id(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated .apollo.hdmap.Id overlap_id = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_overlap_id(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<18>(ptr));
        } else
          goto handle_unusual;
        continue;
      // repeated .apollo.hdmap.Curve position = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_position(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<26>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* SpeedBump::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.hdmap.SpeedBump)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.hdmap.Id id = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::id(this), target, stream);
  }

  // repeated .apollo.hdmap.Id overlap_id = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_overlap_id_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, this->_internal_overlap_id(i), target, stream);
  }

  // repeated .apollo.hdmap.Curve position = 3;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_position_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3, this->_internal_position(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.hdmap.SpeedBump)
  return target;
}

size_t SpeedBump::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.hdmap.SpeedBump)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .apollo.hdmap.Id overlap_id = 2;
  total_size += 1UL * this->_internal_overlap_id_size();
  for (const auto& msg : this->overlap_id_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated .apollo.hdmap.Curve position = 3;
  total_size += 1UL * this->_internal_position_size();
  for (const auto& msg : this->position_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // optional .apollo.hdmap.Id id = 1;
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *id_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData SpeedBump::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    SpeedBump::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*SpeedBump::GetClassData() const { return &_class_data_; }

void SpeedBump::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<SpeedBump *>(to)->MergeFrom(
      static_cast<const SpeedBump &>(from));
}


void SpeedBump::MergeFrom(const SpeedBump& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.hdmap.SpeedBump)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  overlap_id_.MergeFrom(from.overlap_id_);
  position_.MergeFrom(from.position_);
  if (from._internal_has_id()) {
    _internal_mutable_id()->::apollo::hdmap::Id::MergeFrom(from._internal_id());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void SpeedBump::CopyFrom(const SpeedBump& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.hdmap.SpeedBump)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SpeedBump::IsInitialized() const {
  return true;
}

void SpeedBump::InternalSwap(SpeedBump* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  overlap_id_.InternalSwap(&other->overlap_id_);
  position_.InternalSwap(&other->position_);
  swap(id_, other->id_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SpeedBump::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto_getter, &descriptor_table_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto_once,
      file_level_metadata_modules_2fmap_2fproto_2fmap_5fspeed_5fbump_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace hdmap
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::hdmap::SpeedBump* Arena::CreateMaybeMessage< ::apollo::hdmap::SpeedBump >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::hdmap::SpeedBump >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
