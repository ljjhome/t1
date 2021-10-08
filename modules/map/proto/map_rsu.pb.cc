// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/map/proto/map_rsu.proto

#include "modules/map/proto/map_rsu.pb.h"

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
constexpr RSU::RSU(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : overlap_id_()
  , id_(nullptr)
  , junction_id_(nullptr){}
struct RSUDefaultTypeInternal {
  constexpr RSUDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~RSUDefaultTypeInternal() {}
  union {
    RSU _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT RSUDefaultTypeInternal _RSU_default_instance_;
}  // namespace hdmap
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fmap_2fproto_2fmap_5frsu_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fmap_2fproto_2fmap_5frsu_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fmap_2fproto_2fmap_5frsu_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fmap_2fproto_2fmap_5frsu_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::RSU, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::RSU, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::RSU, id_),
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::RSU, junction_id_),
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::RSU, overlap_id_),
  0,
  1,
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, -1, sizeof(::apollo::hdmap::RSU)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::hdmap::_RSU_default_instance_),
};

const char descriptor_table_protodef_modules_2fmap_2fproto_2fmap_5frsu_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\037modules/map/proto/map_rsu.proto\022\014apoll"
  "o.hdmap\032\036modules/map/proto/map_id.proto\""
  "p\n\003RSU\022\034\n\002id\030\001 \001(\0132\020.apollo.hdmap.Id\022%\n\013"
  "junction_id\030\002 \001(\0132\020.apollo.hdmap.Id\022$\n\no"
  "verlap_id\030\003 \003(\0132\020.apollo.hdmap.Id"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fmap_2fproto_2fmap_5frsu_2eproto_deps[1] = {
  &::descriptor_table_modules_2fmap_2fproto_2fmap_5fid_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fmap_2fproto_2fmap_5frsu_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fmap_2fproto_2fmap_5frsu_2eproto = {
  false, false, 193, descriptor_table_protodef_modules_2fmap_2fproto_2fmap_5frsu_2eproto, "modules/map/proto/map_rsu.proto", 
  &descriptor_table_modules_2fmap_2fproto_2fmap_5frsu_2eproto_once, descriptor_table_modules_2fmap_2fproto_2fmap_5frsu_2eproto_deps, 1, 1,
  schemas, file_default_instances, TableStruct_modules_2fmap_2fproto_2fmap_5frsu_2eproto::offsets,
  file_level_metadata_modules_2fmap_2fproto_2fmap_5frsu_2eproto, file_level_enum_descriptors_modules_2fmap_2fproto_2fmap_5frsu_2eproto, file_level_service_descriptors_modules_2fmap_2fproto_2fmap_5frsu_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fmap_2fproto_2fmap_5frsu_2eproto_getter() {
  return &descriptor_table_modules_2fmap_2fproto_2fmap_5frsu_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fmap_2fproto_2fmap_5frsu_2eproto(&descriptor_table_modules_2fmap_2fproto_2fmap_5frsu_2eproto);
namespace apollo {
namespace hdmap {

// ===================================================================

class RSU::_Internal {
 public:
  using HasBits = decltype(std::declval<RSU>()._has_bits_);
  static const ::apollo::hdmap::Id& id(const RSU* msg);
  static void set_has_id(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::apollo::hdmap::Id& junction_id(const RSU* msg);
  static void set_has_junction_id(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

const ::apollo::hdmap::Id&
RSU::_Internal::id(const RSU* msg) {
  return *msg->id_;
}
const ::apollo::hdmap::Id&
RSU::_Internal::junction_id(const RSU* msg) {
  return *msg->junction_id_;
}
void RSU::clear_id() {
  if (id_ != nullptr) id_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void RSU::clear_junction_id() {
  if (junction_id_ != nullptr) junction_id_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
void RSU::clear_overlap_id() {
  overlap_id_.Clear();
}
RSU::RSU(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  overlap_id_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.hdmap.RSU)
}
RSU::RSU(const RSU& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      overlap_id_(from.overlap_id_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_id()) {
    id_ = new ::apollo::hdmap::Id(*from.id_);
  } else {
    id_ = nullptr;
  }
  if (from._internal_has_junction_id()) {
    junction_id_ = new ::apollo::hdmap::Id(*from.junction_id_);
  } else {
    junction_id_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.hdmap.RSU)
}

void RSU::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&id_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&junction_id_) -
    reinterpret_cast<char*>(&id_)) + sizeof(junction_id_));
}

RSU::~RSU() {
  // @@protoc_insertion_point(destructor:apollo.hdmap.RSU)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void RSU::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete id_;
  if (this != internal_default_instance()) delete junction_id_;
}

void RSU::ArenaDtor(void* object) {
  RSU* _this = reinterpret_cast< RSU* >(object);
  (void)_this;
}
void RSU::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void RSU::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void RSU::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.hdmap.RSU)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  overlap_id_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(id_ != nullptr);
      id_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(junction_id_ != nullptr);
      junction_id_->Clear();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* RSU::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // optional .apollo.hdmap.Id junction_id = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_junction_id(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated .apollo.hdmap.Id overlap_id = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_overlap_id(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* RSU::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.hdmap.RSU)
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

  // optional .apollo.hdmap.Id junction_id = 2;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::junction_id(this), target, stream);
  }

  // repeated .apollo.hdmap.Id overlap_id = 3;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_overlap_id_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3, this->_internal_overlap_id(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.hdmap.RSU)
  return target;
}

size_t RSU::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.hdmap.RSU)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .apollo.hdmap.Id overlap_id = 3;
  total_size += 1UL * this->_internal_overlap_id_size();
  for (const auto& msg : this->overlap_id_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    // optional .apollo.hdmap.Id id = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *id_);
    }

    // optional .apollo.hdmap.Id junction_id = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *junction_id_);
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData RSU::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    RSU::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*RSU::GetClassData() const { return &_class_data_; }

void RSU::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<RSU *>(to)->MergeFrom(
      static_cast<const RSU &>(from));
}


void RSU::MergeFrom(const RSU& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.hdmap.RSU)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  overlap_id_.MergeFrom(from.overlap_id_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_id()->::apollo::hdmap::Id::MergeFrom(from._internal_id());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_junction_id()->::apollo::hdmap::Id::MergeFrom(from._internal_junction_id());
    }
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void RSU::CopyFrom(const RSU& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.hdmap.RSU)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RSU::IsInitialized() const {
  return true;
}

void RSU::InternalSwap(RSU* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  overlap_id_.InternalSwap(&other->overlap_id_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(RSU, junction_id_)
      + sizeof(RSU::junction_id_)
      - PROTOBUF_FIELD_OFFSET(RSU, id_)>(
          reinterpret_cast<char*>(&id_),
          reinterpret_cast<char*>(&other->id_));
}

::PROTOBUF_NAMESPACE_ID::Metadata RSU::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fmap_2fproto_2fmap_5frsu_2eproto_getter, &descriptor_table_modules_2fmap_2fproto_2fmap_5frsu_2eproto_once,
      file_level_metadata_modules_2fmap_2fproto_2fmap_5frsu_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace hdmap
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::hdmap::RSU* Arena::CreateMaybeMessage< ::apollo::hdmap::RSU >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::hdmap::RSU >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
