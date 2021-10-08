// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/planning/proto/lattice_structure.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto

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
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[2]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto;
namespace apollo {
namespace planning {
class PlanningTarget;
struct PlanningTargetDefaultTypeInternal;
extern PlanningTargetDefaultTypeInternal _PlanningTarget_default_instance_;
class StopPoint;
struct StopPointDefaultTypeInternal;
extern StopPointDefaultTypeInternal _StopPoint_default_instance_;
}  // namespace planning
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::planning::PlanningTarget* Arena::CreateMaybeMessage<::apollo::planning::PlanningTarget>(Arena*);
template<> ::apollo::planning::StopPoint* Arena::CreateMaybeMessage<::apollo::planning::StopPoint>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace planning {

enum StopPoint_Type : int {
  StopPoint_Type_HARD = 0,
  StopPoint_Type_SOFT = 1
};
bool StopPoint_Type_IsValid(int value);
constexpr StopPoint_Type StopPoint_Type_Type_MIN = StopPoint_Type_HARD;
constexpr StopPoint_Type StopPoint_Type_Type_MAX = StopPoint_Type_SOFT;
constexpr int StopPoint_Type_Type_ARRAYSIZE = StopPoint_Type_Type_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* StopPoint_Type_descriptor();
template<typename T>
inline const std::string& StopPoint_Type_Name(T enum_t_value) {
  static_assert(::std::is_same<T, StopPoint_Type>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function StopPoint_Type_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    StopPoint_Type_descriptor(), enum_t_value);
}
inline bool StopPoint_Type_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, StopPoint_Type* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<StopPoint_Type>(
    StopPoint_Type_descriptor(), name, value);
}
// ===================================================================

class StopPoint final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.planning.StopPoint) */ {
 public:
  inline StopPoint() : StopPoint(nullptr) {}
  ~StopPoint() override;
  explicit constexpr StopPoint(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  StopPoint(const StopPoint& from);
  StopPoint(StopPoint&& from) noexcept
    : StopPoint() {
    *this = ::std::move(from);
  }

  inline StopPoint& operator=(const StopPoint& from) {
    CopyFrom(from);
    return *this;
  }
  inline StopPoint& operator=(StopPoint&& from) noexcept {
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
  static const StopPoint& default_instance() {
    return *internal_default_instance();
  }
  static inline const StopPoint* internal_default_instance() {
    return reinterpret_cast<const StopPoint*>(
               &_StopPoint_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(StopPoint& a, StopPoint& b) {
    a.Swap(&b);
  }
  inline void Swap(StopPoint* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(StopPoint* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline StopPoint* New() const final {
    return new StopPoint();
  }

  StopPoint* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<StopPoint>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const StopPoint& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const StopPoint& from);
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
  void InternalSwap(StopPoint* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.planning.StopPoint";
  }
  protected:
  explicit StopPoint(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  typedef StopPoint_Type Type;
  static constexpr Type HARD =
    StopPoint_Type_HARD;
  static constexpr Type SOFT =
    StopPoint_Type_SOFT;
  static inline bool Type_IsValid(int value) {
    return StopPoint_Type_IsValid(value);
  }
  static constexpr Type Type_MIN =
    StopPoint_Type_Type_MIN;
  static constexpr Type Type_MAX =
    StopPoint_Type_Type_MAX;
  static constexpr int Type_ARRAYSIZE =
    StopPoint_Type_Type_ARRAYSIZE;
  static inline const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor*
  Type_descriptor() {
    return StopPoint_Type_descriptor();
  }
  template<typename T>
  static inline const std::string& Type_Name(T enum_t_value) {
    static_assert(::std::is_same<T, Type>::value ||
      ::std::is_integral<T>::value,
      "Incorrect type passed to function Type_Name.");
    return StopPoint_Type_Name(enum_t_value);
  }
  static inline bool Type_Parse(::PROTOBUF_NAMESPACE_ID::ConstStringParam name,
      Type* value) {
    return StopPoint_Type_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  enum : int {
    kSFieldNumber = 1,
    kTypeFieldNumber = 2,
  };
  // optional double s = 1;
  bool has_s() const;
  private:
  bool _internal_has_s() const;
  public:
  void clear_s();
  double s() const;
  void set_s(double value);
  private:
  double _internal_s() const;
  void _internal_set_s(double value);
  public:

  // optional .apollo.planning.StopPoint.Type type = 2 [default = HARD];
  bool has_type() const;
  private:
  bool _internal_has_type() const;
  public:
  void clear_type();
  ::apollo::planning::StopPoint_Type type() const;
  void set_type(::apollo::planning::StopPoint_Type value);
  private:
  ::apollo::planning::StopPoint_Type _internal_type() const;
  void _internal_set_type(::apollo::planning::StopPoint_Type value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.planning.StopPoint)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  double s_;
  int type_;
  friend struct ::TableStruct_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto;
};
// -------------------------------------------------------------------

class PlanningTarget final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.planning.PlanningTarget) */ {
 public:
  inline PlanningTarget() : PlanningTarget(nullptr) {}
  ~PlanningTarget() override;
  explicit constexpr PlanningTarget(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  PlanningTarget(const PlanningTarget& from);
  PlanningTarget(PlanningTarget&& from) noexcept
    : PlanningTarget() {
    *this = ::std::move(from);
  }

  inline PlanningTarget& operator=(const PlanningTarget& from) {
    CopyFrom(from);
    return *this;
  }
  inline PlanningTarget& operator=(PlanningTarget&& from) noexcept {
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
  static const PlanningTarget& default_instance() {
    return *internal_default_instance();
  }
  static inline const PlanningTarget* internal_default_instance() {
    return reinterpret_cast<const PlanningTarget*>(
               &_PlanningTarget_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(PlanningTarget& a, PlanningTarget& b) {
    a.Swap(&b);
  }
  inline void Swap(PlanningTarget* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(PlanningTarget* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PlanningTarget* New() const final {
    return new PlanningTarget();
  }

  PlanningTarget* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PlanningTarget>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const PlanningTarget& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const PlanningTarget& from);
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
  void InternalSwap(PlanningTarget* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.planning.PlanningTarget";
  }
  protected:
  explicit PlanningTarget(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kStopPointFieldNumber = 1,
    kCruiseSpeedFieldNumber = 2,
  };
  // optional .apollo.planning.StopPoint stop_point = 1;
  bool has_stop_point() const;
  private:
  bool _internal_has_stop_point() const;
  public:
  void clear_stop_point();
  const ::apollo::planning::StopPoint& stop_point() const;
  PROTOBUF_MUST_USE_RESULT ::apollo::planning::StopPoint* release_stop_point();
  ::apollo::planning::StopPoint* mutable_stop_point();
  void set_allocated_stop_point(::apollo::planning::StopPoint* stop_point);
  private:
  const ::apollo::planning::StopPoint& _internal_stop_point() const;
  ::apollo::planning::StopPoint* _internal_mutable_stop_point();
  public:
  void unsafe_arena_set_allocated_stop_point(
      ::apollo::planning::StopPoint* stop_point);
  ::apollo::planning::StopPoint* unsafe_arena_release_stop_point();

  // optional double cruise_speed = 2;
  bool has_cruise_speed() const;
  private:
  bool _internal_has_cruise_speed() const;
  public:
  void clear_cruise_speed();
  double cruise_speed() const;
  void set_cruise_speed(double value);
  private:
  double _internal_cruise_speed() const;
  void _internal_set_cruise_speed(double value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.planning.PlanningTarget)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::apollo::planning::StopPoint* stop_point_;
  double cruise_speed_;
  friend struct ::TableStruct_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// StopPoint

// optional double s = 1;
inline bool StopPoint::_internal_has_s() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool StopPoint::has_s() const {
  return _internal_has_s();
}
inline void StopPoint::clear_s() {
  s_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline double StopPoint::_internal_s() const {
  return s_;
}
inline double StopPoint::s() const {
  // @@protoc_insertion_point(field_get:apollo.planning.StopPoint.s)
  return _internal_s();
}
inline void StopPoint::_internal_set_s(double value) {
  _has_bits_[0] |= 0x00000001u;
  s_ = value;
}
inline void StopPoint::set_s(double value) {
  _internal_set_s(value);
  // @@protoc_insertion_point(field_set:apollo.planning.StopPoint.s)
}

// optional .apollo.planning.StopPoint.Type type = 2 [default = HARD];
inline bool StopPoint::_internal_has_type() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool StopPoint::has_type() const {
  return _internal_has_type();
}
inline void StopPoint::clear_type() {
  type_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::apollo::planning::StopPoint_Type StopPoint::_internal_type() const {
  return static_cast< ::apollo::planning::StopPoint_Type >(type_);
}
inline ::apollo::planning::StopPoint_Type StopPoint::type() const {
  // @@protoc_insertion_point(field_get:apollo.planning.StopPoint.type)
  return _internal_type();
}
inline void StopPoint::_internal_set_type(::apollo::planning::StopPoint_Type value) {
  assert(::apollo::planning::StopPoint_Type_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  type_ = value;
}
inline void StopPoint::set_type(::apollo::planning::StopPoint_Type value) {
  _internal_set_type(value);
  // @@protoc_insertion_point(field_set:apollo.planning.StopPoint.type)
}

// -------------------------------------------------------------------

// PlanningTarget

// optional .apollo.planning.StopPoint stop_point = 1;
inline bool PlanningTarget::_internal_has_stop_point() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || stop_point_ != nullptr);
  return value;
}
inline bool PlanningTarget::has_stop_point() const {
  return _internal_has_stop_point();
}
inline void PlanningTarget::clear_stop_point() {
  if (stop_point_ != nullptr) stop_point_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::planning::StopPoint& PlanningTarget::_internal_stop_point() const {
  const ::apollo::planning::StopPoint* p = stop_point_;
  return p != nullptr ? *p : reinterpret_cast<const ::apollo::planning::StopPoint&>(
      ::apollo::planning::_StopPoint_default_instance_);
}
inline const ::apollo::planning::StopPoint& PlanningTarget::stop_point() const {
  // @@protoc_insertion_point(field_get:apollo.planning.PlanningTarget.stop_point)
  return _internal_stop_point();
}
inline void PlanningTarget::unsafe_arena_set_allocated_stop_point(
    ::apollo::planning::StopPoint* stop_point) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(stop_point_);
  }
  stop_point_ = stop_point;
  if (stop_point) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:apollo.planning.PlanningTarget.stop_point)
}
inline ::apollo::planning::StopPoint* PlanningTarget::release_stop_point() {
  _has_bits_[0] &= ~0x00000001u;
  ::apollo::planning::StopPoint* temp = stop_point_;
  stop_point_ = nullptr;
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
inline ::apollo::planning::StopPoint* PlanningTarget::unsafe_arena_release_stop_point() {
  // @@protoc_insertion_point(field_release:apollo.planning.PlanningTarget.stop_point)
  _has_bits_[0] &= ~0x00000001u;
  ::apollo::planning::StopPoint* temp = stop_point_;
  stop_point_ = nullptr;
  return temp;
}
inline ::apollo::planning::StopPoint* PlanningTarget::_internal_mutable_stop_point() {
  _has_bits_[0] |= 0x00000001u;
  if (stop_point_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::planning::StopPoint>(GetArenaForAllocation());
    stop_point_ = p;
  }
  return stop_point_;
}
inline ::apollo::planning::StopPoint* PlanningTarget::mutable_stop_point() {
  ::apollo::planning::StopPoint* _msg = _internal_mutable_stop_point();
  // @@protoc_insertion_point(field_mutable:apollo.planning.PlanningTarget.stop_point)
  return _msg;
}
inline void PlanningTarget::set_allocated_stop_point(::apollo::planning::StopPoint* stop_point) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete stop_point_;
  }
  if (stop_point) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<::apollo::planning::StopPoint>::GetOwningArena(stop_point);
    if (message_arena != submessage_arena) {
      stop_point = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, stop_point, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  stop_point_ = stop_point;
  // @@protoc_insertion_point(field_set_allocated:apollo.planning.PlanningTarget.stop_point)
}

// optional double cruise_speed = 2;
inline bool PlanningTarget::_internal_has_cruise_speed() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool PlanningTarget::has_cruise_speed() const {
  return _internal_has_cruise_speed();
}
inline void PlanningTarget::clear_cruise_speed() {
  cruise_speed_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline double PlanningTarget::_internal_cruise_speed() const {
  return cruise_speed_;
}
inline double PlanningTarget::cruise_speed() const {
  // @@protoc_insertion_point(field_get:apollo.planning.PlanningTarget.cruise_speed)
  return _internal_cruise_speed();
}
inline void PlanningTarget::_internal_set_cruise_speed(double value) {
  _has_bits_[0] |= 0x00000002u;
  cruise_speed_ = value;
}
inline void PlanningTarget::set_cruise_speed(double value) {
  _internal_set_cruise_speed(value);
  // @@protoc_insertion_point(field_set:apollo.planning.PlanningTarget.cruise_speed)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace planning
}  // namespace apollo

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::apollo::planning::StopPoint_Type> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::planning::StopPoint_Type>() {
  return ::apollo::planning::StopPoint_Type_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto
