// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/map/proto/map_clear_area.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3018000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3018000 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "modules/map/proto/map_id.pb.h"
#include "modules/map/proto/map_geometry.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto;
namespace apollo {
namespace hdmap {
class ClearArea;
struct ClearAreaDefaultTypeInternal;
extern ClearAreaDefaultTypeInternal _ClearArea_default_instance_;
}  // namespace hdmap
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::hdmap::ClearArea* Arena::CreateMaybeMessage<::apollo::hdmap::ClearArea>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace hdmap {

// ===================================================================

class ClearArea final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.hdmap.ClearArea) */ {
 public:
  inline ClearArea() : ClearArea(nullptr) {}
  ~ClearArea() override;
  explicit constexpr ClearArea(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  ClearArea(const ClearArea& from);
  ClearArea(ClearArea&& from) noexcept
    : ClearArea() {
    *this = ::std::move(from);
  }

  inline ClearArea& operator=(const ClearArea& from) {
    CopyFrom(from);
    return *this;
  }
  inline ClearArea& operator=(ClearArea&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const ClearArea& default_instance() {
    return *internal_default_instance();
  }
  static inline const ClearArea* internal_default_instance() {
    return reinterpret_cast<const ClearArea*>(
               &_ClearArea_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ClearArea& a, ClearArea& b) {
    a.Swap(&b);
  }
  inline void Swap(ClearArea* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(ClearArea* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ClearArea* New() const final {
    return new ClearArea();
  }

  ClearArea* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ClearArea>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const ClearArea& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const ClearArea& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to, const ::PROTOBUF_NAMESPACE_ID::Message& from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(ClearArea* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.hdmap.ClearArea";
  }
  protected:
  explicit ClearArea(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kOverlapIdFieldNumber = 2,
    kIdFieldNumber = 1,
    kPolygonFieldNumber = 3,
  };
  // repeated .apollo.hdmap.Id overlap_id = 2;
  int overlap_id_size() const;
  private:
  int _internal_overlap_id_size() const;
  public:
  void clear_overlap_id();
  ::apollo::hdmap::Id* mutable_overlap_id(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::Id >*
      mutable_overlap_id();
  private:
  const ::apollo::hdmap::Id& _internal_overlap_id(int index) const;
  ::apollo::hdmap::Id* _internal_add_overlap_id();
  public:
  const ::apollo::hdmap::Id& overlap_id(int index) const;
  ::apollo::hdmap::Id* add_overlap_id();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::Id >&
      overlap_id() const;

  // optional .apollo.hdmap.Id id = 1;
  bool has_id() const;
  private:
  bool _internal_has_id() const;
  public:
  void clear_id();
  const ::apollo::hdmap::Id& id() const;
  PROTOBUF_MUST_USE_RESULT ::apollo::hdmap::Id* release_id();
  ::apollo::hdmap::Id* mutable_id();
  void set_allocated_id(::apollo::hdmap::Id* id);
  private:
  const ::apollo::hdmap::Id& _internal_id() const;
  ::apollo::hdmap::Id* _internal_mutable_id();
  public:
  void unsafe_arena_set_allocated_id(
      ::apollo::hdmap::Id* id);
  ::apollo::hdmap::Id* unsafe_arena_release_id();

  // optional .apollo.hdmap.Polygon polygon = 3;
  bool has_polygon() const;
  private:
  bool _internal_has_polygon() const;
  public:
  void clear_polygon();
  const ::apollo::hdmap::Polygon& polygon() const;
  PROTOBUF_MUST_USE_RESULT ::apollo::hdmap::Polygon* release_polygon();
  ::apollo::hdmap::Polygon* mutable_polygon();
  void set_allocated_polygon(::apollo::hdmap::Polygon* polygon);
  private:
  const ::apollo::hdmap::Polygon& _internal_polygon() const;
  ::apollo::hdmap::Polygon* _internal_mutable_polygon();
  public:
  void unsafe_arena_set_allocated_polygon(
      ::apollo::hdmap::Polygon* polygon);
  ::apollo::hdmap::Polygon* unsafe_arena_release_polygon();

  // @@protoc_insertion_point(class_scope:apollo.hdmap.ClearArea)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::Id > overlap_id_;
  ::apollo::hdmap::Id* id_;
  ::apollo::hdmap::Polygon* polygon_;
  friend struct ::TableStruct_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ClearArea

// optional .apollo.hdmap.Id id = 1;
inline bool ClearArea::_internal_has_id() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || id_ != nullptr);
  return value;
}
inline bool ClearArea::has_id() const {
  return _internal_has_id();
}
inline const ::apollo::hdmap::Id& ClearArea::_internal_id() const {
  const ::apollo::hdmap::Id* p = id_;
  return p != nullptr ? *p : reinterpret_cast<const ::apollo::hdmap::Id&>(
      ::apollo::hdmap::_Id_default_instance_);
}
inline const ::apollo::hdmap::Id& ClearArea::id() const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.ClearArea.id)
  return _internal_id();
}
inline void ClearArea::unsafe_arena_set_allocated_id(
    ::apollo::hdmap::Id* id) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(id_);
  }
  id_ = id;
  if (id) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:apollo.hdmap.ClearArea.id)
}
inline ::apollo::hdmap::Id* ClearArea::release_id() {
  _has_bits_[0] &= ~0x00000001u;
  ::apollo::hdmap::Id* temp = id_;
  id_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::apollo::hdmap::Id* ClearArea::unsafe_arena_release_id() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.ClearArea.id)
  _has_bits_[0] &= ~0x00000001u;
  ::apollo::hdmap::Id* temp = id_;
  id_ = nullptr;
  return temp;
}
inline ::apollo::hdmap::Id* ClearArea::_internal_mutable_id() {
  _has_bits_[0] |= 0x00000001u;
  if (id_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::hdmap::Id>(GetArenaForAllocation());
    id_ = p;
  }
  return id_;
}
inline ::apollo::hdmap::Id* ClearArea::mutable_id() {
  ::apollo::hdmap::Id* _msg = _internal_mutable_id();
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.ClearArea.id)
  return _msg;
}
inline void ClearArea::set_allocated_id(::apollo::hdmap::Id* id) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(id_);
  }
  if (id) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(id));
    if (message_arena != submessage_arena) {
      id = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, id, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  id_ = id;
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.ClearArea.id)
}

// repeated .apollo.hdmap.Id overlap_id = 2;
inline int ClearArea::_internal_overlap_id_size() const {
  return overlap_id_.size();
}
inline int ClearArea::overlap_id_size() const {
  return _internal_overlap_id_size();
}
inline ::apollo::hdmap::Id* ClearArea::mutable_overlap_id(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.ClearArea.overlap_id)
  return overlap_id_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::Id >*
ClearArea::mutable_overlap_id() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.ClearArea.overlap_id)
  return &overlap_id_;
}
inline const ::apollo::hdmap::Id& ClearArea::_internal_overlap_id(int index) const {
  return overlap_id_.Get(index);
}
inline const ::apollo::hdmap::Id& ClearArea::overlap_id(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.ClearArea.overlap_id)
  return _internal_overlap_id(index);
}
inline ::apollo::hdmap::Id* ClearArea::_internal_add_overlap_id() {
  return overlap_id_.Add();
}
inline ::apollo::hdmap::Id* ClearArea::add_overlap_id() {
  ::apollo::hdmap::Id* _add = _internal_add_overlap_id();
  // @@protoc_insertion_point(field_add:apollo.hdmap.ClearArea.overlap_id)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::Id >&
ClearArea::overlap_id() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.ClearArea.overlap_id)
  return overlap_id_;
}

// optional .apollo.hdmap.Polygon polygon = 3;
inline bool ClearArea::_internal_has_polygon() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || polygon_ != nullptr);
  return value;
}
inline bool ClearArea::has_polygon() const {
  return _internal_has_polygon();
}
inline const ::apollo::hdmap::Polygon& ClearArea::_internal_polygon() const {
  const ::apollo::hdmap::Polygon* p = polygon_;
  return p != nullptr ? *p : reinterpret_cast<const ::apollo::hdmap::Polygon&>(
      ::apollo::hdmap::_Polygon_default_instance_);
}
inline const ::apollo::hdmap::Polygon& ClearArea::polygon() const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.ClearArea.polygon)
  return _internal_polygon();
}
inline void ClearArea::unsafe_arena_set_allocated_polygon(
    ::apollo::hdmap::Polygon* polygon) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(polygon_);
  }
  polygon_ = polygon;
  if (polygon) {
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:apollo.hdmap.ClearArea.polygon)
}
inline ::apollo::hdmap::Polygon* ClearArea::release_polygon() {
  _has_bits_[0] &= ~0x00000002u;
  ::apollo::hdmap::Polygon* temp = polygon_;
  polygon_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::apollo::hdmap::Polygon* ClearArea::unsafe_arena_release_polygon() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.ClearArea.polygon)
  _has_bits_[0] &= ~0x00000002u;
  ::apollo::hdmap::Polygon* temp = polygon_;
  polygon_ = nullptr;
  return temp;
}
inline ::apollo::hdmap::Polygon* ClearArea::_internal_mutable_polygon() {
  _has_bits_[0] |= 0x00000002u;
  if (polygon_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::hdmap::Polygon>(GetArenaForAllocation());
    polygon_ = p;
  }
  return polygon_;
}
inline ::apollo::hdmap::Polygon* ClearArea::mutable_polygon() {
  ::apollo::hdmap::Polygon* _msg = _internal_mutable_polygon();
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.ClearArea.polygon)
  return _msg;
}
inline void ClearArea::set_allocated_polygon(::apollo::hdmap::Polygon* polygon) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(polygon_);
  }
  if (polygon) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(polygon));
    if (message_arena != submessage_arena) {
      polygon = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, polygon, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  polygon_ = polygon;
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.ClearArea.polygon)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fclear_5farea_2eproto
