// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/drivers/microphone/proto/microphone_config.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fdrivers_2fmicrophone_2fproto_2fmicrophone_5fconfig_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fdrivers_2fmicrophone_2fproto_2fmicrophone_5fconfig_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_modules_2fdrivers_2fmicrophone_2fproto_2fmicrophone_5fconfig_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fdrivers_2fmicrophone_2fproto_2fmicrophone_5fconfig_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fdrivers_2fmicrophone_2fproto_2fmicrophone_5fconfig_2eproto;
namespace apollo {
namespace drivers {
namespace microphone {
namespace config {
class MicrophoneConfig;
struct MicrophoneConfigDefaultTypeInternal;
extern MicrophoneConfigDefaultTypeInternal _MicrophoneConfig_default_instance_;
}  // namespace config
}  // namespace microphone
}  // namespace drivers
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::drivers::microphone::config::MicrophoneConfig* Arena::CreateMaybeMessage<::apollo::drivers::microphone::config::MicrophoneConfig>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace drivers {
namespace microphone {
namespace config {

enum MicrophoneConfig_MicrophoneModel : int {
  MicrophoneConfig_MicrophoneModel_UNKNOWN = 0,
  MicrophoneConfig_MicrophoneModel_RESPEAKER = 1
};
bool MicrophoneConfig_MicrophoneModel_IsValid(int value);
constexpr MicrophoneConfig_MicrophoneModel MicrophoneConfig_MicrophoneModel_MicrophoneModel_MIN = MicrophoneConfig_MicrophoneModel_UNKNOWN;
constexpr MicrophoneConfig_MicrophoneModel MicrophoneConfig_MicrophoneModel_MicrophoneModel_MAX = MicrophoneConfig_MicrophoneModel_RESPEAKER;
constexpr int MicrophoneConfig_MicrophoneModel_MicrophoneModel_ARRAYSIZE = MicrophoneConfig_MicrophoneModel_MicrophoneModel_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* MicrophoneConfig_MicrophoneModel_descriptor();
template<typename T>
inline const std::string& MicrophoneConfig_MicrophoneModel_Name(T enum_t_value) {
  static_assert(::std::is_same<T, MicrophoneConfig_MicrophoneModel>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function MicrophoneConfig_MicrophoneModel_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    MicrophoneConfig_MicrophoneModel_descriptor(), enum_t_value);
}
inline bool MicrophoneConfig_MicrophoneModel_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, MicrophoneConfig_MicrophoneModel* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<MicrophoneConfig_MicrophoneModel>(
    MicrophoneConfig_MicrophoneModel_descriptor(), name, value);
}
enum ChannelType : int {
  UNKNOWN = 0,
  ASR = 1,
  RAW = 2,
  PLAYBACK = 3
};
bool ChannelType_IsValid(int value);
constexpr ChannelType ChannelType_MIN = UNKNOWN;
constexpr ChannelType ChannelType_MAX = PLAYBACK;
constexpr int ChannelType_ARRAYSIZE = ChannelType_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* ChannelType_descriptor();
template<typename T>
inline const std::string& ChannelType_Name(T enum_t_value) {
  static_assert(::std::is_same<T, ChannelType>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function ChannelType_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    ChannelType_descriptor(), enum_t_value);
}
inline bool ChannelType_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, ChannelType* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<ChannelType>(
    ChannelType_descriptor(), name, value);
}
// ===================================================================

class MicrophoneConfig final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.drivers.microphone.config.MicrophoneConfig) */ {
 public:
  inline MicrophoneConfig() : MicrophoneConfig(nullptr) {}
  ~MicrophoneConfig() override;
  explicit constexpr MicrophoneConfig(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  MicrophoneConfig(const MicrophoneConfig& from);
  MicrophoneConfig(MicrophoneConfig&& from) noexcept
    : MicrophoneConfig() {
    *this = ::std::move(from);
  }

  inline MicrophoneConfig& operator=(const MicrophoneConfig& from) {
    CopyFrom(from);
    return *this;
  }
  inline MicrophoneConfig& operator=(MicrophoneConfig&& from) noexcept {
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
  static const MicrophoneConfig& default_instance() {
    return *internal_default_instance();
  }
  static inline const MicrophoneConfig* internal_default_instance() {
    return reinterpret_cast<const MicrophoneConfig*>(
               &_MicrophoneConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(MicrophoneConfig& a, MicrophoneConfig& b) {
    a.Swap(&b);
  }
  inline void Swap(MicrophoneConfig* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(MicrophoneConfig* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MicrophoneConfig* New() const final {
    return new MicrophoneConfig();
  }

  MicrophoneConfig* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MicrophoneConfig>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const MicrophoneConfig& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const MicrophoneConfig& from);
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
  void InternalSwap(MicrophoneConfig* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.drivers.microphone.config.MicrophoneConfig";
  }
  protected:
  explicit MicrophoneConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  typedef MicrophoneConfig_MicrophoneModel MicrophoneModel;
  static constexpr MicrophoneModel UNKNOWN =
    MicrophoneConfig_MicrophoneModel_UNKNOWN;
  static constexpr MicrophoneModel RESPEAKER =
    MicrophoneConfig_MicrophoneModel_RESPEAKER;
  static inline bool MicrophoneModel_IsValid(int value) {
    return MicrophoneConfig_MicrophoneModel_IsValid(value);
  }
  static constexpr MicrophoneModel MicrophoneModel_MIN =
    MicrophoneConfig_MicrophoneModel_MicrophoneModel_MIN;
  static constexpr MicrophoneModel MicrophoneModel_MAX =
    MicrophoneConfig_MicrophoneModel_MicrophoneModel_MAX;
  static constexpr int MicrophoneModel_ARRAYSIZE =
    MicrophoneConfig_MicrophoneModel_MicrophoneModel_ARRAYSIZE;
  static inline const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor*
  MicrophoneModel_descriptor() {
    return MicrophoneConfig_MicrophoneModel_descriptor();
  }
  template<typename T>
  static inline const std::string& MicrophoneModel_Name(T enum_t_value) {
    static_assert(::std::is_same<T, MicrophoneModel>::value ||
      ::std::is_integral<T>::value,
      "Incorrect type passed to function MicrophoneModel_Name.");
    return MicrophoneConfig_MicrophoneModel_Name(enum_t_value);
  }
  static inline bool MicrophoneModel_Parse(::PROTOBUF_NAMESPACE_ID::ConstStringParam name,
      MicrophoneModel* value) {
    return MicrophoneConfig_MicrophoneModel_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  enum : int {
    kChannelTypeFieldNumber = 1,
    kChannelNameFieldNumber = 7,
    kFrameIdFieldNumber = 8,
    kMicrophoneModelFieldNumber = 2,
    kChunkFieldNumber = 3,
    kSampleRateFieldNumber = 4,
    kRecordSecondsFieldNumber = 5,
    kSampleWidthFieldNumber = 6,
    kMicDistanceFieldNumber = 9,
  };
  // repeated .apollo.drivers.microphone.config.ChannelType channel_type = 1;
  int channel_type_size() const;
  private:
  int _internal_channel_type_size() const;
  public:
  void clear_channel_type();
  private:
  ::apollo::drivers::microphone::config::ChannelType _internal_channel_type(int index) const;
  void _internal_add_channel_type(::apollo::drivers::microphone::config::ChannelType value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>* _internal_mutable_channel_type();
  public:
  ::apollo::drivers::microphone::config::ChannelType channel_type(int index) const;
  void set_channel_type(int index, ::apollo::drivers::microphone::config::ChannelType value);
  void add_channel_type(::apollo::drivers::microphone::config::ChannelType value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>& channel_type() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>* mutable_channel_type();

  // optional string channel_name = 7;
  bool has_channel_name() const;
  private:
  bool _internal_has_channel_name() const;
  public:
  void clear_channel_name();
  const std::string& channel_name() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_channel_name(ArgT0&& arg0, ArgT... args);
  std::string* mutable_channel_name();
  PROTOBUF_MUST_USE_RESULT std::string* release_channel_name();
  void set_allocated_channel_name(std::string* channel_name);
  private:
  const std::string& _internal_channel_name() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_channel_name(const std::string& value);
  std::string* _internal_mutable_channel_name();
  public:

  // optional string frame_id = 8;
  bool has_frame_id() const;
  private:
  bool _internal_has_frame_id() const;
  public:
  void clear_frame_id();
  const std::string& frame_id() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_frame_id(ArgT0&& arg0, ArgT... args);
  std::string* mutable_frame_id();
  PROTOBUF_MUST_USE_RESULT std::string* release_frame_id();
  void set_allocated_frame_id(std::string* frame_id);
  private:
  const std::string& _internal_frame_id() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_frame_id(const std::string& value);
  std::string* _internal_mutable_frame_id();
  public:

  // optional .apollo.drivers.microphone.config.MicrophoneConfig.MicrophoneModel microphone_model = 2;
  bool has_microphone_model() const;
  private:
  bool _internal_has_microphone_model() const;
  public:
  void clear_microphone_model();
  ::apollo::drivers::microphone::config::MicrophoneConfig_MicrophoneModel microphone_model() const;
  void set_microphone_model(::apollo::drivers::microphone::config::MicrophoneConfig_MicrophoneModel value);
  private:
  ::apollo::drivers::microphone::config::MicrophoneConfig_MicrophoneModel _internal_microphone_model() const;
  void _internal_set_microphone_model(::apollo::drivers::microphone::config::MicrophoneConfig_MicrophoneModel value);
  public:

  // optional int32 chunk = 3;
  bool has_chunk() const;
  private:
  bool _internal_has_chunk() const;
  public:
  void clear_chunk();
  ::PROTOBUF_NAMESPACE_ID::int32 chunk() const;
  void set_chunk(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_chunk() const;
  void _internal_set_chunk(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // optional float sample_rate = 4;
  bool has_sample_rate() const;
  private:
  bool _internal_has_sample_rate() const;
  public:
  void clear_sample_rate();
  float sample_rate() const;
  void set_sample_rate(float value);
  private:
  float _internal_sample_rate() const;
  void _internal_set_sample_rate(float value);
  public:

  // optional float record_seconds = 5;
  bool has_record_seconds() const;
  private:
  bool _internal_has_record_seconds() const;
  public:
  void clear_record_seconds();
  float record_seconds() const;
  void set_record_seconds(float value);
  private:
  float _internal_record_seconds() const;
  void _internal_set_record_seconds(float value);
  public:

  // optional int32 sample_width = 6;
  bool has_sample_width() const;
  private:
  bool _internal_has_sample_width() const;
  public:
  void clear_sample_width();
  ::PROTOBUF_NAMESPACE_ID::int32 sample_width() const;
  void set_sample_width(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_sample_width() const;
  void _internal_set_sample_width(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // optional float mic_distance = 9;
  bool has_mic_distance() const;
  private:
  bool _internal_has_mic_distance() const;
  public:
  void clear_mic_distance();
  float mic_distance() const;
  void set_mic_distance(float value);
  private:
  float _internal_mic_distance() const;
  void _internal_set_mic_distance(float value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.drivers.microphone.config.MicrophoneConfig)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField<int> channel_type_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr channel_name_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr frame_id_;
  int microphone_model_;
  ::PROTOBUF_NAMESPACE_ID::int32 chunk_;
  float sample_rate_;
  float record_seconds_;
  ::PROTOBUF_NAMESPACE_ID::int32 sample_width_;
  float mic_distance_;
  friend struct ::TableStruct_modules_2fdrivers_2fmicrophone_2fproto_2fmicrophone_5fconfig_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MicrophoneConfig

// optional .apollo.drivers.microphone.config.MicrophoneConfig.MicrophoneModel microphone_model = 2;
inline bool MicrophoneConfig::_internal_has_microphone_model() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool MicrophoneConfig::has_microphone_model() const {
  return _internal_has_microphone_model();
}
inline void MicrophoneConfig::clear_microphone_model() {
  microphone_model_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::apollo::drivers::microphone::config::MicrophoneConfig_MicrophoneModel MicrophoneConfig::_internal_microphone_model() const {
  return static_cast< ::apollo::drivers::microphone::config::MicrophoneConfig_MicrophoneModel >(microphone_model_);
}
inline ::apollo::drivers::microphone::config::MicrophoneConfig_MicrophoneModel MicrophoneConfig::microphone_model() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.microphone.config.MicrophoneConfig.microphone_model)
  return _internal_microphone_model();
}
inline void MicrophoneConfig::_internal_set_microphone_model(::apollo::drivers::microphone::config::MicrophoneConfig_MicrophoneModel value) {
  assert(::apollo::drivers::microphone::config::MicrophoneConfig_MicrophoneModel_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  microphone_model_ = value;
}
inline void MicrophoneConfig::set_microphone_model(::apollo::drivers::microphone::config::MicrophoneConfig_MicrophoneModel value) {
  _internal_set_microphone_model(value);
  // @@protoc_insertion_point(field_set:apollo.drivers.microphone.config.MicrophoneConfig.microphone_model)
}

// optional int32 chunk = 3;
inline bool MicrophoneConfig::_internal_has_chunk() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool MicrophoneConfig::has_chunk() const {
  return _internal_has_chunk();
}
inline void MicrophoneConfig::clear_chunk() {
  chunk_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 MicrophoneConfig::_internal_chunk() const {
  return chunk_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 MicrophoneConfig::chunk() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.microphone.config.MicrophoneConfig.chunk)
  return _internal_chunk();
}
inline void MicrophoneConfig::_internal_set_chunk(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000008u;
  chunk_ = value;
}
inline void MicrophoneConfig::set_chunk(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_chunk(value);
  // @@protoc_insertion_point(field_set:apollo.drivers.microphone.config.MicrophoneConfig.chunk)
}

// optional float sample_rate = 4;
inline bool MicrophoneConfig::_internal_has_sample_rate() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool MicrophoneConfig::has_sample_rate() const {
  return _internal_has_sample_rate();
}
inline void MicrophoneConfig::clear_sample_rate() {
  sample_rate_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline float MicrophoneConfig::_internal_sample_rate() const {
  return sample_rate_;
}
inline float MicrophoneConfig::sample_rate() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.microphone.config.MicrophoneConfig.sample_rate)
  return _internal_sample_rate();
}
inline void MicrophoneConfig::_internal_set_sample_rate(float value) {
  _has_bits_[0] |= 0x00000010u;
  sample_rate_ = value;
}
inline void MicrophoneConfig::set_sample_rate(float value) {
  _internal_set_sample_rate(value);
  // @@protoc_insertion_point(field_set:apollo.drivers.microphone.config.MicrophoneConfig.sample_rate)
}

// optional float record_seconds = 5;
inline bool MicrophoneConfig::_internal_has_record_seconds() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool MicrophoneConfig::has_record_seconds() const {
  return _internal_has_record_seconds();
}
inline void MicrophoneConfig::clear_record_seconds() {
  record_seconds_ = 0;
  _has_bits_[0] &= ~0x00000020u;
}
inline float MicrophoneConfig::_internal_record_seconds() const {
  return record_seconds_;
}
inline float MicrophoneConfig::record_seconds() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.microphone.config.MicrophoneConfig.record_seconds)
  return _internal_record_seconds();
}
inline void MicrophoneConfig::_internal_set_record_seconds(float value) {
  _has_bits_[0] |= 0x00000020u;
  record_seconds_ = value;
}
inline void MicrophoneConfig::set_record_seconds(float value) {
  _internal_set_record_seconds(value);
  // @@protoc_insertion_point(field_set:apollo.drivers.microphone.config.MicrophoneConfig.record_seconds)
}

// optional int32 sample_width = 6;
inline bool MicrophoneConfig::_internal_has_sample_width() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool MicrophoneConfig::has_sample_width() const {
  return _internal_has_sample_width();
}
inline void MicrophoneConfig::clear_sample_width() {
  sample_width_ = 0;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 MicrophoneConfig::_internal_sample_width() const {
  return sample_width_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 MicrophoneConfig::sample_width() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.microphone.config.MicrophoneConfig.sample_width)
  return _internal_sample_width();
}
inline void MicrophoneConfig::_internal_set_sample_width(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000040u;
  sample_width_ = value;
}
inline void MicrophoneConfig::set_sample_width(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_sample_width(value);
  // @@protoc_insertion_point(field_set:apollo.drivers.microphone.config.MicrophoneConfig.sample_width)
}

// optional string channel_name = 7;
inline bool MicrophoneConfig::_internal_has_channel_name() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool MicrophoneConfig::has_channel_name() const {
  return _internal_has_channel_name();
}
inline void MicrophoneConfig::clear_channel_name() {
  channel_name_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& MicrophoneConfig::channel_name() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.microphone.config.MicrophoneConfig.channel_name)
  return _internal_channel_name();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void MicrophoneConfig::set_channel_name(ArgT0&& arg0, ArgT... args) {
 _has_bits_[0] |= 0x00000001u;
 channel_name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:apollo.drivers.microphone.config.MicrophoneConfig.channel_name)
}
inline std::string* MicrophoneConfig::mutable_channel_name() {
  std::string* _s = _internal_mutable_channel_name();
  // @@protoc_insertion_point(field_mutable:apollo.drivers.microphone.config.MicrophoneConfig.channel_name)
  return _s;
}
inline const std::string& MicrophoneConfig::_internal_channel_name() const {
  return channel_name_.Get();
}
inline void MicrophoneConfig::_internal_set_channel_name(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  channel_name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* MicrophoneConfig::_internal_mutable_channel_name() {
  _has_bits_[0] |= 0x00000001u;
  return channel_name_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* MicrophoneConfig::release_channel_name() {
  // @@protoc_insertion_point(field_release:apollo.drivers.microphone.config.MicrophoneConfig.channel_name)
  if (!_internal_has_channel_name()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return channel_name_.ReleaseNonDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void MicrophoneConfig::set_allocated_channel_name(std::string* channel_name) {
  if (channel_name != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  channel_name_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), channel_name,
      GetArenaForAllocation());
  // @@protoc_insertion_point(field_set_allocated:apollo.drivers.microphone.config.MicrophoneConfig.channel_name)
}

// optional string frame_id = 8;
inline bool MicrophoneConfig::_internal_has_frame_id() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool MicrophoneConfig::has_frame_id() const {
  return _internal_has_frame_id();
}
inline void MicrophoneConfig::clear_frame_id() {
  frame_id_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000002u;
}
inline const std::string& MicrophoneConfig::frame_id() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.microphone.config.MicrophoneConfig.frame_id)
  return _internal_frame_id();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void MicrophoneConfig::set_frame_id(ArgT0&& arg0, ArgT... args) {
 _has_bits_[0] |= 0x00000002u;
 frame_id_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:apollo.drivers.microphone.config.MicrophoneConfig.frame_id)
}
inline std::string* MicrophoneConfig::mutable_frame_id() {
  std::string* _s = _internal_mutable_frame_id();
  // @@protoc_insertion_point(field_mutable:apollo.drivers.microphone.config.MicrophoneConfig.frame_id)
  return _s;
}
inline const std::string& MicrophoneConfig::_internal_frame_id() const {
  return frame_id_.Get();
}
inline void MicrophoneConfig::_internal_set_frame_id(const std::string& value) {
  _has_bits_[0] |= 0x00000002u;
  frame_id_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* MicrophoneConfig::_internal_mutable_frame_id() {
  _has_bits_[0] |= 0x00000002u;
  return frame_id_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* MicrophoneConfig::release_frame_id() {
  // @@protoc_insertion_point(field_release:apollo.drivers.microphone.config.MicrophoneConfig.frame_id)
  if (!_internal_has_frame_id()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000002u;
  return frame_id_.ReleaseNonDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void MicrophoneConfig::set_allocated_frame_id(std::string* frame_id) {
  if (frame_id != nullptr) {
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  frame_id_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), frame_id,
      GetArenaForAllocation());
  // @@protoc_insertion_point(field_set_allocated:apollo.drivers.microphone.config.MicrophoneConfig.frame_id)
}

// optional float mic_distance = 9;
inline bool MicrophoneConfig::_internal_has_mic_distance() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool MicrophoneConfig::has_mic_distance() const {
  return _internal_has_mic_distance();
}
inline void MicrophoneConfig::clear_mic_distance() {
  mic_distance_ = 0;
  _has_bits_[0] &= ~0x00000080u;
}
inline float MicrophoneConfig::_internal_mic_distance() const {
  return mic_distance_;
}
inline float MicrophoneConfig::mic_distance() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.microphone.config.MicrophoneConfig.mic_distance)
  return _internal_mic_distance();
}
inline void MicrophoneConfig::_internal_set_mic_distance(float value) {
  _has_bits_[0] |= 0x00000080u;
  mic_distance_ = value;
}
inline void MicrophoneConfig::set_mic_distance(float value) {
  _internal_set_mic_distance(value);
  // @@protoc_insertion_point(field_set:apollo.drivers.microphone.config.MicrophoneConfig.mic_distance)
}

// repeated .apollo.drivers.microphone.config.ChannelType channel_type = 1;
inline int MicrophoneConfig::_internal_channel_type_size() const {
  return channel_type_.size();
}
inline int MicrophoneConfig::channel_type_size() const {
  return _internal_channel_type_size();
}
inline void MicrophoneConfig::clear_channel_type() {
  channel_type_.Clear();
}
inline ::apollo::drivers::microphone::config::ChannelType MicrophoneConfig::_internal_channel_type(int index) const {
  return static_cast< ::apollo::drivers::microphone::config::ChannelType >(channel_type_.Get(index));
}
inline ::apollo::drivers::microphone::config::ChannelType MicrophoneConfig::channel_type(int index) const {
  // @@protoc_insertion_point(field_get:apollo.drivers.microphone.config.MicrophoneConfig.channel_type)
  return _internal_channel_type(index);
}
inline void MicrophoneConfig::set_channel_type(int index, ::apollo::drivers::microphone::config::ChannelType value) {
  assert(::apollo::drivers::microphone::config::ChannelType_IsValid(value));
  channel_type_.Set(index, value);
  // @@protoc_insertion_point(field_set:apollo.drivers.microphone.config.MicrophoneConfig.channel_type)
}
inline void MicrophoneConfig::_internal_add_channel_type(::apollo::drivers::microphone::config::ChannelType value) {
  assert(::apollo::drivers::microphone::config::ChannelType_IsValid(value));
  channel_type_.Add(value);
}
inline void MicrophoneConfig::add_channel_type(::apollo::drivers::microphone::config::ChannelType value) {
  _internal_add_channel_type(value);
  // @@protoc_insertion_point(field_add:apollo.drivers.microphone.config.MicrophoneConfig.channel_type)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>&
MicrophoneConfig::channel_type() const {
  // @@protoc_insertion_point(field_list:apollo.drivers.microphone.config.MicrophoneConfig.channel_type)
  return channel_type_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>*
MicrophoneConfig::_internal_mutable_channel_type() {
  return &channel_type_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField<int>*
MicrophoneConfig::mutable_channel_type() {
  // @@protoc_insertion_point(field_mutable_list:apollo.drivers.microphone.config.MicrophoneConfig.channel_type)
  return _internal_mutable_channel_type();
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace config
}  // namespace microphone
}  // namespace drivers
}  // namespace apollo

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::apollo::drivers::microphone::config::MicrophoneConfig_MicrophoneModel> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::drivers::microphone::config::MicrophoneConfig_MicrophoneModel>() {
  return ::apollo::drivers::microphone::config::MicrophoneConfig_MicrophoneModel_descriptor();
}
template <> struct is_proto_enum< ::apollo::drivers::microphone::config::ChannelType> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::drivers::microphone::config::ChannelType>() {
  return ::apollo::drivers::microphone::config::ChannelType_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fdrivers_2fmicrophone_2fproto_2fmicrophone_5fconfig_2eproto
