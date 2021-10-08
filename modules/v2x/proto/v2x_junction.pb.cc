// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/v2x/proto/v2x_junction.proto

#include "modules/v2x/proto/v2x_junction.pb.h"

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
namespace v2x {
constexpr Id::Id(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : id_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string){}
struct IdDefaultTypeInternal {
  constexpr IdDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~IdDefaultTypeInternal() {}
  union {
    Id _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT IdDefaultTypeInternal _Id_default_instance_;
constexpr Junction::Junction(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : crosswalk_id_()
  , overlap_id_()
  , lane_id_()
  , edge_type_()
  , id_(nullptr)
  , polygon_(nullptr)
  , type_(0)

  , num_road_segments_(4){}
struct JunctionDefaultTypeInternal {
  constexpr JunctionDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~JunctionDefaultTypeInternal() {}
  union {
    Junction _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT JunctionDefaultTypeInternal _Junction_default_instance_;
}  // namespace v2x
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto[2];
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::Id, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::Id, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::Id, id_),
  0,
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::Junction, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::Junction, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::Junction, id_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::Junction, polygon_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::Junction, crosswalk_id_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::Junction, overlap_id_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::Junction, num_road_segments_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::Junction, lane_id_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::Junction, type_),
  PROTOBUF_FIELD_OFFSET(::apollo::v2x::Junction, edge_type_),
  0,
  1,
  ~0u,
  ~0u,
  3,
  ~0u,
  2,
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, -1, sizeof(::apollo::v2x::Id)},
  { 8, 22, -1, sizeof(::apollo::v2x::Junction)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::v2x::_Id_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::v2x::_Junction_default_instance_),
};

const char descriptor_table_protodef_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n$modules/v2x/proto/v2x_junction.proto\022\n"
  "apollo.v2x\032#modules/common/proto/geometr"
  "y.proto\"\020\n\002Id\022\n\n\002id\030\001 \001(\014\"\214\003\n\010Junction\022\032"
  "\n\002id\030\001 \001(\0132\016.apollo.v2x.Id\022\'\n\007polygon\030\002 "
  "\001(\0132\026.apollo.common.Polygon\022$\n\014crosswalk"
  "_id\030\003 \003(\0132\016.apollo.v2x.Id\022\"\n\noverlap_id\030"
  "\004 \003(\0132\016.apollo.v2x.Id\022\034\n\021num_road_segmen"
  "ts\030\005 \001(\005:\0014\022\037\n\007lane_id\030\006 \003(\0132\016.apollo.v2"
  "x.Id\022\'\n\004type\030\007 \001(\0162\031.apollo.v2x.Junction"
  ".Type\0220\n\tedge_type\030\010 \003(\0162\035.apollo.v2x.Ju"
  "nction.EdgeType\"0\n\004Type\022\013\n\007UNKNOWN\020\000\022\013\n\007"
  "IN_ROAD\020\001\022\016\n\nCROSS_ROAD\020\002\"%\n\010EdgeType\022\014\n"
  "\010PHYSICAL\020\000\022\013\n\007VIRTUAL\020\001"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto_deps[1] = {
  &::descriptor_table_modules_2fcommon_2fproto_2fgeometry_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto = {
  false, false, 504, descriptor_table_protodef_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto, "modules/v2x/proto/v2x_junction.proto", 
  &descriptor_table_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto_once, descriptor_table_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto_deps, 1, 2,
  schemas, file_default_instances, TableStruct_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto::offsets,
  file_level_metadata_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto, file_level_enum_descriptors_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto, file_level_service_descriptors_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto_getter() {
  return &descriptor_table_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto(&descriptor_table_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto);
namespace apollo {
namespace v2x {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Junction_Type_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto);
  return file_level_enum_descriptors_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto[0];
}
bool Junction_Type_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
      return true;
    default:
      return false;
  }
}

#if (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)
constexpr Junction_Type Junction::UNKNOWN;
constexpr Junction_Type Junction::IN_ROAD;
constexpr Junction_Type Junction::CROSS_ROAD;
constexpr Junction_Type Junction::Type_MIN;
constexpr Junction_Type Junction::Type_MAX;
constexpr int Junction::Type_ARRAYSIZE;
#endif  // (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Junction_EdgeType_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto);
  return file_level_enum_descriptors_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto[1];
}
bool Junction_EdgeType_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
      return true;
    default:
      return false;
  }
}

#if (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)
constexpr Junction_EdgeType Junction::PHYSICAL;
constexpr Junction_EdgeType Junction::VIRTUAL;
constexpr Junction_EdgeType Junction::EdgeType_MIN;
constexpr Junction_EdgeType Junction::EdgeType_MAX;
constexpr int Junction::EdgeType_ARRAYSIZE;
#endif  // (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)

// ===================================================================

class Id::_Internal {
 public:
  using HasBits = decltype(std::declval<Id>()._has_bits_);
  static void set_has_id(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

Id::Id(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.v2x.Id)
}
Id::Id(const Id& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  id_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_id()) {
    id_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_id(), 
      GetArenaForAllocation());
  }
  // @@protoc_insertion_point(copy_constructor:apollo.v2x.Id)
}

void Id::SharedCtor() {
id_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

Id::~Id() {
  // @@protoc_insertion_point(destructor:apollo.v2x.Id)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Id::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  id_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void Id::ArenaDtor(void* object) {
  Id* _this = reinterpret_cast< Id* >(object);
  (void)_this;
}
void Id::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Id::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Id::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.v2x.Id)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    id_.ClearNonDefaultToEmpty();
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Id::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional bytes id = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          auto str = _internal_mutable_id();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
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

::PROTOBUF_NAMESPACE_ID::uint8* Id::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.v2x.Id)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional bytes id = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->WriteBytesMaybeAliased(
        1, this->_internal_id(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.v2x.Id)
  return target;
}

size_t Id::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.v2x.Id)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // optional bytes id = 1;
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::BytesSize(
        this->_internal_id());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Id::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Id::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Id::GetClassData() const { return &_class_data_; }

void Id::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Id *>(to)->MergeFrom(
      static_cast<const Id &>(from));
}


void Id::MergeFrom(const Id& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.v2x.Id)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_id()) {
    _internal_set_id(from._internal_id());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Id::CopyFrom(const Id& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.v2x.Id)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Id::IsInitialized() const {
  return true;
}

void Id::InternalSwap(Id* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &id_, lhs_arena,
      &other->id_, rhs_arena
  );
}

::PROTOBUF_NAMESPACE_ID::Metadata Id::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto_getter, &descriptor_table_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto_once,
      file_level_metadata_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto[0]);
}

// ===================================================================

class Junction::_Internal {
 public:
  using HasBits = decltype(std::declval<Junction>()._has_bits_);
  static const ::apollo::v2x::Id& id(const Junction* msg);
  static void set_has_id(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::apollo::common::Polygon& polygon(const Junction* msg);
  static void set_has_polygon(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_num_road_segments(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_type(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::apollo::v2x::Id&
Junction::_Internal::id(const Junction* msg) {
  return *msg->id_;
}
const ::apollo::common::Polygon&
Junction::_Internal::polygon(const Junction* msg) {
  return *msg->polygon_;
}
void Junction::clear_polygon() {
  if (polygon_ != nullptr) polygon_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
Junction::Junction(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  crosswalk_id_(arena),
  overlap_id_(arena),
  lane_id_(arena),
  edge_type_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.v2x.Junction)
}
Junction::Junction(const Junction& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      crosswalk_id_(from.crosswalk_id_),
      overlap_id_(from.overlap_id_),
      lane_id_(from.lane_id_),
      edge_type_(from.edge_type_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_id()) {
    id_ = new ::apollo::v2x::Id(*from.id_);
  } else {
    id_ = nullptr;
  }
  if (from._internal_has_polygon()) {
    polygon_ = new ::apollo::common::Polygon(*from.polygon_);
  } else {
    polygon_ = nullptr;
  }
  ::memcpy(&type_, &from.type_,
    static_cast<size_t>(reinterpret_cast<char*>(&num_road_segments_) -
    reinterpret_cast<char*>(&type_)) + sizeof(num_road_segments_));
  // @@protoc_insertion_point(copy_constructor:apollo.v2x.Junction)
}

void Junction::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&id_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&type_) -
    reinterpret_cast<char*>(&id_)) + sizeof(type_));
num_road_segments_ = 4;
}

Junction::~Junction() {
  // @@protoc_insertion_point(destructor:apollo.v2x.Junction)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Junction::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete id_;
  if (this != internal_default_instance()) delete polygon_;
}

void Junction::ArenaDtor(void* object) {
  Junction* _this = reinterpret_cast< Junction* >(object);
  (void)_this;
}
void Junction::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Junction::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Junction::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.v2x.Junction)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  crosswalk_id_.Clear();
  overlap_id_.Clear();
  lane_id_.Clear();
  edge_type_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(id_ != nullptr);
      id_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(polygon_ != nullptr);
      polygon_->Clear();
    }
  }
  if (cached_has_bits & 0x0000000cu) {
    type_ = 0;
    num_road_segments_ = 4;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Junction::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional .apollo.v2x.Id id = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_id(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional .apollo.common.Polygon polygon = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_polygon(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated .apollo.v2x.Id crosswalk_id = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_crosswalk_id(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<26>(ptr));
        } else
          goto handle_unusual;
        continue;
      // repeated .apollo.v2x.Id overlap_id = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 34)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_overlap_id(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<34>(ptr));
        } else
          goto handle_unusual;
        continue;
      // optional int32 num_road_segments = 5 [default = 4];
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 40)) {
          _Internal::set_has_num_road_segments(&has_bits);
          num_road_segments_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated .apollo.v2x.Id lane_id = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 50)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_lane_id(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<50>(ptr));
        } else
          goto handle_unusual;
        continue;
      // optional .apollo.v2x.Junction.Type type = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 56)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::apollo::v2x::Junction_Type_IsValid(val))) {
            _internal_set_type(static_cast<::apollo::v2x::Junction_Type>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(7, val, mutable_unknown_fields());
          }
        } else
          goto handle_unusual;
        continue;
      // repeated .apollo.v2x.Junction.EdgeType edge_type = 8;
      case 8:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 64)) {
          ptr -= 1;
          do {
            ptr += 1;
            ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
            CHK_(ptr);
            if (PROTOBUF_PREDICT_TRUE(::apollo::v2x::Junction_EdgeType_IsValid(val))) {
              _internal_add_edge_type(static_cast<::apollo::v2x::Junction_EdgeType>(val));
            } else {
              ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(8, val, mutable_unknown_fields());
            }
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<64>(ptr));
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 66) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedEnumParser<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(_internal_mutable_edge_type(), ptr, ctx, ::apollo::v2x::Junction_EdgeType_IsValid, &_internal_metadata_, 8);
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

::PROTOBUF_NAMESPACE_ID::uint8* Junction::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.v2x.Junction)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.v2x.Id id = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::id(this), target, stream);
  }

  // optional .apollo.common.Polygon polygon = 2;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::polygon(this), target, stream);
  }

  // repeated .apollo.v2x.Id crosswalk_id = 3;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_crosswalk_id_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3, this->_internal_crosswalk_id(i), target, stream);
  }

  // repeated .apollo.v2x.Id overlap_id = 4;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_overlap_id_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(4, this->_internal_overlap_id(i), target, stream);
  }

  // optional int32 num_road_segments = 5 [default = 4];
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(5, this->_internal_num_road_segments(), target);
  }

  // repeated .apollo.v2x.Id lane_id = 6;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_lane_id_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(6, this->_internal_lane_id(i), target, stream);
  }

  // optional .apollo.v2x.Junction.Type type = 7;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      7, this->_internal_type(), target);
  }

  // repeated .apollo.v2x.Junction.EdgeType edge_type = 8;
  for (int i = 0, n = this->_internal_edge_type_size(); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
        8, this->_internal_edge_type(i), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.v2x.Junction)
  return target;
}

size_t Junction::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.v2x.Junction)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .apollo.v2x.Id crosswalk_id = 3;
  total_size += 1UL * this->_internal_crosswalk_id_size();
  for (const auto& msg : this->crosswalk_id_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated .apollo.v2x.Id overlap_id = 4;
  total_size += 1UL * this->_internal_overlap_id_size();
  for (const auto& msg : this->overlap_id_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated .apollo.v2x.Id lane_id = 6;
  total_size += 1UL * this->_internal_lane_id_size();
  for (const auto& msg : this->lane_id_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated .apollo.v2x.Junction.EdgeType edge_type = 8;
  {
    size_t data_size = 0;
    unsigned int count = static_cast<unsigned int>(this->_internal_edge_type_size());for (unsigned int i = 0; i < count; i++) {
      data_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(
        this->_internal_edge_type(static_cast<int>(i)));
    }
    total_size += (1UL * count) + data_size;
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    // optional .apollo.v2x.Id id = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *id_);
    }

    // optional .apollo.common.Polygon polygon = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *polygon_);
    }

    // optional .apollo.v2x.Junction.Type type = 7;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_type());
    }

    // optional int32 num_road_segments = 5 [default = 4];
    if (cached_has_bits & 0x00000008u) {
      total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32SizePlusOne(this->_internal_num_road_segments());
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Junction::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Junction::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Junction::GetClassData() const { return &_class_data_; }

void Junction::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Junction *>(to)->MergeFrom(
      static_cast<const Junction &>(from));
}


void Junction::MergeFrom(const Junction& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.v2x.Junction)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  crosswalk_id_.MergeFrom(from.crosswalk_id_);
  overlap_id_.MergeFrom(from.overlap_id_);
  lane_id_.MergeFrom(from.lane_id_);
  edge_type_.MergeFrom(from.edge_type_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_id()->::apollo::v2x::Id::MergeFrom(from._internal_id());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_polygon()->::apollo::common::Polygon::MergeFrom(from._internal_polygon());
    }
    if (cached_has_bits & 0x00000004u) {
      type_ = from.type_;
    }
    if (cached_has_bits & 0x00000008u) {
      num_road_segments_ = from.num_road_segments_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Junction::CopyFrom(const Junction& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.v2x.Junction)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Junction::IsInitialized() const {
  return true;
}

void Junction::InternalSwap(Junction* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  crosswalk_id_.InternalSwap(&other->crosswalk_id_);
  overlap_id_.InternalSwap(&other->overlap_id_);
  lane_id_.InternalSwap(&other->lane_id_);
  edge_type_.InternalSwap(&other->edge_type_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Junction, type_)
      + sizeof(Junction::type_)
      - PROTOBUF_FIELD_OFFSET(Junction, id_)>(
          reinterpret_cast<char*>(&id_),
          reinterpret_cast<char*>(&other->id_));
  swap(num_road_segments_, other->num_road_segments_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Junction::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto_getter, &descriptor_table_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto_once,
      file_level_metadata_modules_2fv2x_2fproto_2fv2x_5fjunction_2eproto[1]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace v2x
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::v2x::Id* Arena::CreateMaybeMessage< ::apollo::v2x::Id >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::v2x::Id >(arena);
}
template<> PROTOBUF_NOINLINE ::apollo::v2x::Junction* Arena::CreateMaybeMessage< ::apollo::v2x::Junction >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::v2x::Junction >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
