/*
Copyright 2018 Google Inc. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS-IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "graph/resonance_audio_api_impl.h"

#include <algorithm>
#include <numeric>

#include "mysofa.h"

#include "ambisonics/ambisonic_binaural_decoder.h"
#include "ambisonics/ambisonic_codec_impl.h" // TODO: is it okay to use this over the parent class?
#include "ambisonics/utils.h"
#include "base/constants_and_types.h"
#include "base/logging.h"
#include "base/misc_math.h"
#include "base/source_parameters.h"

#include "base/unique_ptr_wrapper.h"
#include "config/source_config.h"
#include "dsp/channel_converter.h"
#include "dsp/distance_attenuation.h"
#include "dsp/sh_hrir_creator.h"
#include "graph/source_parameters_manager.h"
#include "utils/planar_interleaved_conversion.h"
#include "utils/sample_type_conversion.h"
#include "utils/wav.h"

namespace vraudio {

namespace {

// Support 50 setter calls for 512 sources.
const size_t kMaxNumTasksOnTaskQueue = 50 * 512;

// User warning/notification messages.
static const char* kBadInputPointerMessage = "Ignoring nullptr buffer";
static const char* kBufferSizeMustMatchNumFramesMessage =
    "Number of frames must match the frames per buffer specified during "
    "construction - ignoring buffer";

// Helper method to fetch |SourceGraphConfig| from |RenderingMode|.
SourceGraphConfig GetSourceGraphConfigFromRenderingMode(
    RenderingMode rendering_mode) {
  switch (rendering_mode) {
    case RenderingMode::kStereoPanning:
      return StereoPanningConfig();
    case RenderingMode::kBinauralLowQuality:
      return BinauralLowQualityConfig();
    case RenderingMode::kBinauralMediumQuality:
      return BinauralMediumQualityConfig();
    case RenderingMode::kBinauralHighQuality:
      return BinauralHighQualityConfig();
    case RenderingMode::kBinauralHigherQuality:
      return BinauralHigherQualityConfig();
    case RenderingMode::kRoomEffectsOnly:
      return RoomEffectsOnlyConfig();
    default:
      LOG(FATAL) << "Unknown rendering mode";
      break;
  }
  return BinauralHighQualityConfig();
}

}  // namespace

ResonanceAudioApiImpl::ResonanceAudioApiImpl(size_t num_channels,
                                             size_t frames_per_buffer,
                                             int sample_rate_hz)
    : system_settings_(num_channels, frames_per_buffer, sample_rate_hz),
      task_queue_(kMaxNumTasksOnTaskQueue),
      source_id_counter_(0) {
  if (num_channels != kNumStereoChannels) {
    LOG(FATAL) << "Only stereo output is supported";
    return;
  }

  if (frames_per_buffer > kMaxSupportedNumFrames) {
    LOG(FATAL) << "Only frame lengths up to " << kMaxSupportedNumFrames
               << " are supported.";
    return;
  }

  // The pffft library requires a minimum buffer size of 32 samples.
  if (frames_per_buffer < FftManager::kMinFftSize) {
    LOG(FATAL) << "The minimum number of frames per buffer is "
               << FftManager::kMinFftSize << " samples";
    return;
  }
  graph_manager_.reset(new GraphManager(system_settings_));
}

ResonanceAudioApiImpl::~ResonanceAudioApiImpl() {
  // Clear task queue before shutting down.
  task_queue_.Execute();
}

bool ResonanceAudioApiImpl::FillInterleavedOutputBuffer(size_t num_channels,
                                                        size_t num_frames,
                                                        float* buffer_ptr) {
  DCHECK(buffer_ptr);
  return FillOutputBuffer<float*>(num_channels, num_frames, buffer_ptr);
}

bool ResonanceAudioApiImpl::FillInterleavedOutputBuffer(size_t num_channels,
                                                        size_t num_frames,
                                                        int16* buffer_ptr) {
  DCHECK(buffer_ptr);
  return FillOutputBuffer<int16*>(num_channels, num_frames, buffer_ptr);
}

bool ResonanceAudioApiImpl::FillPlanarOutputBuffer(size_t num_channels,
                                                   size_t num_frames,
                                                   float* const* buffer_ptr) {
  DCHECK(buffer_ptr);
  return FillOutputBuffer<float* const*>(num_channels, num_frames, buffer_ptr);
}

bool ResonanceAudioApiImpl::FillPlanarOutputBuffer(size_t num_channels,
                                                   size_t num_frames,
                                                   int16* const* buffer_ptr) {
  DCHECK(buffer_ptr);
  return FillOutputBuffer<int16* const*>(num_channels, num_frames, buffer_ptr);
}

void ResonanceAudioApiImpl::SetHeadPosition(float x, float y, float z) {
  auto task = [this, x, y, z]() {
    const WorldPosition head_position(x, y, z);
    system_settings_.SetHeadPosition(head_position);
  };
  task_queue_.Post(task);
}

void ResonanceAudioApiImpl::SetHeadRotation(float x, float y, float z,
                                            float w) {
  auto task = [this, w, x, y, z]() {
    const WorldRotation head_rotation(w, x, y, z);
    system_settings_.SetHeadRotation(head_rotation);
  };
  task_queue_.Post(task);
}

void ResonanceAudioApiImpl::SetMasterVolume(float volume) {
  auto task = [this, volume]() { system_settings_.SetMasterGain(volume); };
  task_queue_.Post(task);
}

int ResonanceAudioApiImpl::CreateAmbisonicSource(size_t num_channels) {
  if (num_channels < kNumFirstOrderAmbisonicChannels ||
      !IsValidAmbisonicOrder(num_channels)) {
    // Invalid number of input channels, don't create the ambisonic source.
    LOG(ERROR) << "Invalid number of channels for the ambisonic source: "
               << num_channels;
    return kInvalidSourceId;
  }

  const int ambisonic_source_id = source_id_counter_.fetch_add(1);

  const size_t num_valid_channels =
      std::min(num_channels, graph_manager_->GetNumMaxAmbisonicChannels());
  if (num_valid_channels < num_channels) {
    LOG(WARNING) << "Number of ambisonic channels will be diminished to "
                 << num_valid_channels;
  }

  auto task = [this, ambisonic_source_id, num_valid_channels]() {
    graph_manager_->CreateAmbisonicSource(ambisonic_source_id,
                                          num_valid_channels);
    system_settings_.GetSourceParametersManager()->Register(
        ambisonic_source_id);
    // Overwrite default source parameters for ambisonic source.
    auto source_parameters =
        system_settings_.GetSourceParametersManager()->GetMutableParameters(
            ambisonic_source_id);
    source_parameters->room_effects_gain = 0.0f;
    source_parameters->distance_rolloff_model = DistanceRolloffModel::kNone;
    source_parameters->distance_attenuation = 1.0f;
  };
  task_queue_.Post(task);
  return ambisonic_source_id;
}

int ResonanceAudioApiImpl::CreateStereoSource(size_t num_channels) {
  if (num_channels > kNumStereoChannels) {
    LOG(ERROR) << "Unsupported number of input channels";
    return kInvalidSourceId;
  }
  const int stereo_source_id = source_id_counter_.fetch_add(1);

  auto task = [this, stereo_source_id]() {
    graph_manager_->CreateStereoSource(stereo_source_id);
    system_settings_.GetSourceParametersManager()->Register(stereo_source_id);
    auto source_parameters =
        system_settings_.GetSourceParametersManager()->GetMutableParameters(
            stereo_source_id);
    source_parameters->enable_hrtf = false;
  };
  task_queue_.Post(task);
  return stereo_source_id;
}

int ResonanceAudioApiImpl::CreateSoundObjectSource(
    RenderingMode rendering_mode) {
  const int sound_object_source_id = source_id_counter_.fetch_add(1);

  const auto config = GetSourceGraphConfigFromRenderingMode(rendering_mode);
  auto task = [this, sound_object_source_id, config]() {
    graph_manager_->CreateSoundObjectSource(
        sound_object_source_id, config.ambisonic_order, config.enable_hrtf,
        config.enable_direct_rendering);
    system_settings_.GetSourceParametersManager()->Register(
        sound_object_source_id);
    auto source_parameters =
        system_settings_.GetSourceParametersManager()->GetMutableParameters(
            sound_object_source_id);
    source_parameters->enable_hrtf = config.enable_hrtf;
  };
  task_queue_.Post(task);
  return sound_object_source_id;
}

void ResonanceAudioApiImpl::DestroySource(SourceId source_id) {
  auto task = [this, source_id]() {
    graph_manager_->DestroySource(source_id);
    system_settings_.GetSourceParametersManager()->Unregister(source_id);
  };
  task_queue_.Post(task);
}

void ResonanceAudioApiImpl::SetInterleavedBuffer(SourceId source_id,
                                                 const float* audio_buffer_ptr,
                                                 size_t num_channels,
                                                 size_t num_frames) {
  SetSourceBuffer<const float*>(source_id, audio_buffer_ptr, num_channels,
                                num_frames);
}

void ResonanceAudioApiImpl::SetInterleavedBuffer(SourceId source_id,
                                                 const int16* audio_buffer_ptr,
                                                 size_t num_channels,
                                                 size_t num_frames) {
  SetSourceBuffer<const int16*>(source_id, audio_buffer_ptr, num_channels,
                                num_frames);
}

void ResonanceAudioApiImpl::SetPlanarBuffer(
    SourceId source_id, const float* const* audio_buffer_ptr,
    size_t num_channels, size_t num_frames) {
  SetSourceBuffer<const float* const*>(source_id, audio_buffer_ptr,
                                       num_channels, num_frames);
}

void ResonanceAudioApiImpl::SetPlanarBuffer(
    SourceId source_id, const int16* const* audio_buffer_ptr,
    size_t num_channels, size_t num_frames) {
  SetSourceBuffer<const int16* const*>(source_id, audio_buffer_ptr,
                                       num_channels, num_frames);
}

void ResonanceAudioApiImpl::SetSourceDistanceAttenuation(
    SourceId source_id, float distance_attenuation) {
  auto task = [this, source_id, distance_attenuation]() {
    auto source_parameters =
        system_settings_.GetSourceParametersManager()->GetMutableParameters(
            source_id);
    if (source_parameters != nullptr) {
      const auto& rolloff_model = source_parameters->distance_rolloff_model;
      DCHECK_EQ(rolloff_model, DistanceRolloffModel::kNone);
      if (rolloff_model != DistanceRolloffModel::kNone) {
        LOG(WARNING) << "Implicit distance rolloff model is set. The value "
                        "will be overwritten.";
      }
      source_parameters->distance_attenuation = distance_attenuation;
    }
  };
  task_queue_.Post(task);
}

void ResonanceAudioApiImpl::SetSourceDistanceModel(SourceId source_id,
                                                   DistanceRolloffModel rolloff,
                                                   float min_distance,
                                                   float max_distance) {
  if (max_distance < min_distance && rolloff != DistanceRolloffModel::kNone) {
    LOG(WARNING) << "max_distance must be larger than min_distance";
    return;
  }
  auto task = [this, source_id, rolloff, min_distance, max_distance]() {
    auto source_parameters =
        system_settings_.GetSourceParametersManager()->GetMutableParameters(
            source_id);
    if (source_parameters != nullptr) {
      source_parameters->distance_rolloff_model = rolloff;
      source_parameters->minimum_distance = min_distance;
      source_parameters->maximum_distance = max_distance;
    }
  };
  task_queue_.Post(task);
}

void ResonanceAudioApiImpl::SetSourcePosition(SourceId source_id, float x,
                                              float y, float z) {
  const WorldPosition position(x, y, z);
  auto task = [this, source_id, position]() {
    auto source_parameters =
        system_settings_.GetSourceParametersManager()->GetMutableParameters(
            source_id);
    if (source_parameters != nullptr) {
      source_parameters->object_transform.position = position;
    }
  };
  task_queue_.Post(task);
}

void ResonanceAudioApiImpl::SetSourceRoomEffectsGain(SourceId source_id,
                                                     float room_effects_gain) {
  auto task = [this, source_id, room_effects_gain]() {
    auto source_parameters =
        system_settings_.GetSourceParametersManager()->GetMutableParameters(
            source_id);
    if (source_parameters != nullptr) {
      source_parameters->room_effects_gain = room_effects_gain;
    }
  };
  task_queue_.Post(task);
}

void ResonanceAudioApiImpl::SetSourceRotation(SourceId source_id, float x,
                                              float y, float z, float w) {
  const WorldRotation rotation(w, x, y, z);
  auto task = [this, source_id, rotation]() {
    auto source_parameters =
        system_settings_.GetSourceParametersManager()->GetMutableParameters(
            source_id);
    if (source_parameters != nullptr) {
      source_parameters->object_transform.rotation = rotation;
    }
  };
  task_queue_.Post(task);
}

void ResonanceAudioApiImpl::SetSourceVolume(SourceId source_id, float volume) {
  auto task = [this, source_id, volume]() {
    auto source_parameters =
        system_settings_.GetSourceParametersManager()->GetMutableParameters(
            source_id);
    if (source_parameters != nullptr) {
      source_parameters->gain = volume;
    }
  };
  task_queue_.Post(task);
}

void ResonanceAudioApiImpl::SetSoundObjectDirectivity(
    SourceId sound_object_source_id, float alpha, float order) {
  auto task = [this, sound_object_source_id, alpha, order]() {
    auto source_parameters =
        system_settings_.GetSourceParametersManager()->GetMutableParameters(
            sound_object_source_id);
    if (source_parameters != nullptr) {
      source_parameters->directivity_alpha = alpha;
      source_parameters->directivity_order = order;
    }
  };
  task_queue_.Post(task);
}

void ResonanceAudioApiImpl::SetSoundObjectListenerDirectivity(
    SourceId sound_object_source_id, float alpha, float order) {
  auto task = [this, sound_object_source_id, alpha, order]() {
    auto source_parameters =
        system_settings_.GetSourceParametersManager()->GetMutableParameters(
            sound_object_source_id);
    if (source_parameters != nullptr) {
      source_parameters->listener_directivity_alpha = alpha;
      source_parameters->listener_directivity_order = order;
    }
  };
  task_queue_.Post(task);
}

void ResonanceAudioApiImpl::SetSoundObjectNearFieldEffectGain(
    SourceId sound_object_source_id, float gain) {
  auto task = [this, sound_object_source_id, gain]() {
    auto source_parameters =
        system_settings_.GetSourceParametersManager()->GetMutableParameters(
            sound_object_source_id);
    if (source_parameters != nullptr) {
      source_parameters->near_field_gain = gain;
    }
  };
  task_queue_.Post(task);
}

void ResonanceAudioApiImpl::SetSoundObjectOcclusionIntensity(
    SourceId sound_object_source_id, float intensity) {
  auto task = [this, sound_object_source_id, intensity]() {
    auto source_parameters =
        system_settings_.GetSourceParametersManager()->GetMutableParameters(
            sound_object_source_id);
    if (source_parameters != nullptr) {
      source_parameters->occlusion_intensity = intensity;
    }
  };
  task_queue_.Post(task);
}

void ResonanceAudioApiImpl::SetSoundObjectSpread(
    SourceId sound_object_source_id, float spread_deg) {
  auto task = [this, sound_object_source_id, spread_deg]() {
    auto source_parameters =
        system_settings_.GetSourceParametersManager()->GetMutableParameters(
            sound_object_source_id);
    if (source_parameters != nullptr) {
      source_parameters->spread_deg = spread_deg;
    }
  };
  task_queue_.Post(task);
}

void ResonanceAudioApiImpl::EnableRoomEffects(bool enable) {
  auto task = [this, enable]() { graph_manager_->EnableRoomEffects(enable); };
  task_queue_.Post(task);
}

void ResonanceAudioApiImpl::SetReflectionProperties(
    const ReflectionProperties& reflection_properties) {
  auto task = [this, reflection_properties]() {
    system_settings_.SetReflectionProperties(reflection_properties);
  };
  task_queue_.Post(task);
}

void ResonanceAudioApiImpl::SetReverbProperties(
    const ReverbProperties& reverb_properties) {
  auto task = [this, reverb_properties]() {
    system_settings_.SetReverbProperties(reverb_properties);
  };
  task_queue_.Post(task);
}

const AudioBuffer* ResonanceAudioApiImpl::GetAmbisonicOutputBuffer() const {
  return graph_manager_->GetAmbisonicBuffer();
}

const AudioBuffer* ResonanceAudioApiImpl::GetStereoOutputBuffer() const {
  return graph_manager_->GetStereoBuffer();
}

void ResonanceAudioApiImpl::ProcessNextBuffer() {
#if defined(ENABLE_TRACING) && !ION_PRODUCTION
  // This enables tracing on the audio thread.
  auto task = []() { ENABLE_TRACING_ON_CURRENT_THREAD("AudioThread"); };
  task_queue_.Post(task);
#endif  // defined(ENABLE_TRACING) && !ION_PRODUCTION


  task_queue_.Execute();

  // Update room effects only if the pipeline is initialized.
  if (graph_manager_->GetRoomEffectsEnabled()) {
    graph_manager_->UpdateRoomReflections();
    graph_manager_->UpdateRoomReverb();
  }
  // Update source attenuation parameters.
  const auto process = [this](SourceParameters* parameters) {
    const float master_gain = system_settings_.GetMasterGain();
    const auto& listener_position = system_settings_.GetHeadPosition();
    const auto& reflection_properties =
        system_settings_.GetReflectionProperties();
    const auto& reverb_properties = system_settings_.GetReverbProperties();
    UpdateAttenuationParameters(master_gain, reflection_properties.gain,
                                reverb_properties.gain, listener_position,
                                parameters);
  };
  system_settings_.GetSourceParametersManager()->ProcessAllParameters(process);

  graph_manager_->Process();
}

void ResonanceAudioApiImpl::SetStereoSpeakerMode(bool enabled) {
  auto task = [this, enabled]() {
    system_settings_.SetStereoSpeakerMode(enabled);
  };
  task_queue_.Post(task);
}

template <typename OutputType>
bool ResonanceAudioApiImpl::FillOutputBuffer(size_t num_channels,
                                             size_t num_frames,
                                             OutputType buffer_ptr) {


  if (buffer_ptr == nullptr) {
    LOG(WARNING) << kBadInputPointerMessage;
    return false;
  }
  if (num_channels != kNumStereoChannels) {
    LOG(WARNING) << "Output buffer must be stereo";
    return false;
  }
  const size_t num_input_samples = num_frames * num_channels;
  const size_t num_expected_output_samples =
      system_settings_.GetFramesPerBuffer() * system_settings_.GetNumChannels();
  if (num_input_samples != num_expected_output_samples) {
    LOG(WARNING) << "Output buffer size must be " << num_expected_output_samples
                 << " samples";
    return false;
  }

  // Get the processed output buffer.
  ProcessNextBuffer();
  const AudioBuffer* output_buffer = GetStereoOutputBuffer();
  if (output_buffer == nullptr) {
    // This indicates that the graph processing is triggered without having any
    // connected sources.
    return false;
  }

  FillExternalBuffer(*output_buffer, buffer_ptr, num_frames, num_channels);
  return true;
}

template <typename SampleType>
void ResonanceAudioApiImpl::SetSourceBuffer(SourceId source_id,
                                            SampleType audio_buffer_ptr,
                                            size_t num_input_channels,
                                            size_t num_frames) {
  // Execute task queue to ensure newly created sound sources are initialized.
  task_queue_.Execute();

  if (audio_buffer_ptr == nullptr) {
    LOG(WARNING) << kBadInputPointerMessage;
    return;
  }
  if (num_frames != system_settings_.GetFramesPerBuffer()) {
    LOG(WARNING) << kBufferSizeMustMatchNumFramesMessage;
    return;
  }

  AudioBuffer* const output_buffer =
      graph_manager_->GetMutableAudioBuffer(source_id);
  if (output_buffer == nullptr) {
    LOG(WARNING) << "Source audio buffer not found";
    return;
  }
  const size_t num_output_channels = output_buffer->num_channels();

  if (num_input_channels == num_output_channels) {
    FillAudioBuffer(audio_buffer_ptr, num_frames, num_input_channels,
                    output_buffer);

    return;
  }

  if ((num_input_channels == kNumMonoChannels) &&
      (num_output_channels == kNumStereoChannels)) {
    FillAudioBufferWithChannelRemapping(
        audio_buffer_ptr, num_frames, num_input_channels,
        {0, 0} /* channel_map */, output_buffer);
    return;
  }

  if (num_input_channels > num_output_channels) {
    std::vector<size_t> channel_map(num_output_channels);
    // Fill channel map with increasing indices.
    std::iota(std::begin(channel_map), std::end(channel_map), 0);
    FillAudioBufferWithChannelRemapping(audio_buffer_ptr, num_frames,
                                        num_input_channels, channel_map,
                                        output_buffer);
    return;
  }

  LOG(WARNING) << "Number of input channels does not match the number of "
                  "output channels";
}

void ResonanceAudioApiImpl::SetHRIR(char* user_hrir) {
    userHRIR = std::string(user_hrir);
    // Create user HRIR ambisonic decoder
    // userHRIR is a series of hex tokens
    std::istringstream hex_tokens(userHRIR);
    std::string token;
    std::stringstream wav_data_stream;
    // convert to a string
    while (std::getline(hex_tokens, token, ',')) {
        wav_data_stream << (unsigned char)std::stoi(token.substr(2, 2), 0, 16);
    }
    std::string testData = wav_data_stream.str();
    // set istringstream wav_data_stream to new string
    // std::istringstream wav_data_stream(userHRIR);
    std::unique_ptr<const Wav> wav = Wav::CreateOrNull(&wav_data_stream);

    if (wav == NULL) {
        LOG(ERROR) << "User HRIR is malformed!";

        return;
    }

    auto sh_hrirs = vraudio::CreateShHrirsFromWav(*wav, graph_manager_->system_settings_.GetSampleRateHz(), &graph_manager_->resampler_);
    auto sh_order = GetPeriphonicAmbisonicOrder(wav->GetNumChannels());
    graph_manager_->ambisonic_binaural_decoder_node_[sh_order]->user_decoder = new AmbisonicBinauralDecoder(
        *sh_hrirs, graph_manager_->system_settings_.GetFramesPerBuffer(), &graph_manager_->fft_manager_);
};

void ResonanceAudioApiImpl::UseHRIR(bool use_hrir) {
    if (use_hrir == bUseHRIR)
        return;

    bUseHRIR = use_hrir;

    if (bUseHRIR) {
        // Enable user hrir for all orders
        LOG(WARNING) << "ENABLING USER HRIR";
        for (auto node : graph_manager_->ambisonic_binaural_decoder_node_) {
            if (node.second->user_decoder)
                node.second->ambisonic_binaural_decoder_ = node.second->user_decoder;
        }
    }
    else {
        // Disable user hrir for all orders
        LOG(WARNING) << "DISABLING USER HRIR";
        for (auto node : graph_manager_->ambisonic_binaural_decoder_node_) {
            if (node.second->builtin_decoder)
                node.second->ambisonic_binaural_decoder_ = node.second->builtin_decoder;
        }
    }
};

// HACK(will): helper function to manually set HDF5 attributes
void HACK_set_if_null(MYSOFA_ATTRIBUTE *attrib, char *name, char *value) {
    MYSOFA_ATTRIBUTE *curr = attrib;
    while (curr != NULL) {
        if (strcmp(curr->name, name) == 0 && curr->value == NULL) {
            curr->value = _strdup(value);
            break;
        }
        curr = curr->next;
    }
}

// HACK(will): manually supply the missing values in netcdf 4.3.1.1 files
void HACK_fix_attrib(MYSOFA_EASY* hrtf) {
    HACK_set_if_null(hrtf->hrtf->ListenerPosition.attributes, "Units", "metre");
    HACK_set_if_null(hrtf->hrtf->ListenerPosition.attributes, "Type", "cartesian");
    HACK_set_if_null(hrtf->hrtf->ReceiverPosition.attributes, "Units", "metre");
    HACK_set_if_null(hrtf->hrtf->ReceiverPosition.attributes, "Type", "cartesian");
    HACK_set_if_null(hrtf->hrtf->SourcePosition.attributes, "Units", "degree, degree, metre");
    HACK_set_if_null(hrtf->hrtf->SourcePosition.attributes, "Type", "spherical");
    HACK_set_if_null(hrtf->hrtf->EmitterPosition.attributes, "Units", "metre");
    HACK_set_if_null(hrtf->hrtf->EmitterPosition.attributes, "Type", "cartesian");
    HACK_set_if_null(hrtf->hrtf->ListenerView.attributes, "Units", "metre");
    HACK_set_if_null(hrtf->hrtf->ListenerView.attributes, "Type", "cartesian");
    HACK_set_if_null(hrtf->hrtf->DataSamplingRate.attributes, "Units", "hertz");
}

bool ResonanceAudioApiImpl::SetCustomSofa(const char* file_name, int ambisonic_order) {
    int err;
    struct MYSOFA_EASY *hrtf = (MYSOFA_EASY*)malloc(sizeof(struct MYSOFA_EASY));
    if (!hrtf)
        return false;

    hrtf->fir = NULL;
    hrtf->lookup = NULL;
    hrtf->neighborhood = NULL;

    // TODO(will): pass in ambisonic speaker positions (acceptable angles) as argument

    // TODO(will): check against settings variable for supported orders instead of hard-coding this
    if (ambisonic_order < 1 || ambisonic_order > 5 || ambisonic_order == 4)
        return false;

    if (file_name == NULL || file_name == "") {
        return false;
    }

    hrtf->hrtf = mysofa_load(file_name, &err);

    if (!hrtf->hrtf) {
        mysofa_close(hrtf);
        return false;
    }

    HACK_fix_attrib(hrtf); // TODO: remove once netcdf 4.3.1.1 handling is fixed

    err = mysofa_check(hrtf->hrtf);
    if (err != MYSOFA_OK) {
        mysofa_close(hrtf);
        return err;
    }

    //mysofa_tocartesian(hrtf->hrtf); // TODO: do we need this?

    // TODO: do we need this if we're just pulling out data?
    //hrtf->lookup = mysofa_lookup_init(hrtf->hrtf);
    //if (hrtf->lookup == NULL) {
    //    err = MYSOFA_INTERNAL_ERROR;
    //    mysofa_close(hrtf);
    //    return err;
    //}
    //hrtf->neighborhood = mysofa_neighborhood_init(hrtf->hrtf, hrtf->lookup);

    int source_sampling_rate = (int)hrtf->hrtf->DataSamplingRate.values[0];

    int numEmitters = (int)hrtf->hrtf->M;
    int numReceivers = (int)hrtf->hrtf->R;
    int numSamples = (int)hrtf->hrtf->N;

    if (numEmitters <= 0 || numReceivers != 2 || numSamples <= 0)
        return false;

    std::vector<float> pos(hrtf->hrtf->SourcePosition.values, 
        hrtf->hrtf->SourcePosition.values + hrtf->hrtf->SourcePosition.elements);

    // NOTE(will): get all the angles and format them for Resonance
    std::vector<float> data(hrtf->hrtf->DataIR.values, 
        hrtf->hrtf->DataIR.values + hrtf->hrtf->DataIR.elements);

    std::vector<SphericalAngle> sofa_angles(numEmitters * numReceivers);
    Eigen::MatrixXf hrir_full_matrix(numSamples, numEmitters * numReceivers);
    int totalSamplesPerEmitter = numSamples * numReceivers;

    for (int i = 0; i < numEmitters; ++i) {
        // TODO(will): filter out angles based on speaker positions
        sofa_angles[i * 2] = SphericalAngle::FromDegrees(pos[i * 3], pos[i * 3 + 1]);
        sofa_angles[i * 2 + 1] = SphericalAngle::FromDegrees(-pos[i * 3], pos[i * 3 + 1]);

        // TODO(will): (opt?) apply fade out to all HRIRs
        // NOTE(will): assumes two receiver positions for now
        for (int n = 0; n < numSamples; ++n) {
            hrir_full_matrix(n, i * 2) = data[i*totalSamplesPerEmitter + n]; // Left channel
            hrir_full_matrix(n, (i * 2) + 1) = data[i*totalSamplesPerEmitter + numSamples + n]; // Right channel
        }
    }

    AmbisonicCodecImpl<> ambisonic_codec(ambisonic_order, sofa_angles);
    auto decoder_matrix = ambisonic_codec.GetDecoderMatrix();
    auto sh = hrir_full_matrix * decoder_matrix;

    //printf("HRIR_full_matrix: %d x %d\n", (int)hrir_full_matrix.rows(), (int)hrir_full_matrix.cols());
    //std::cout << hrir_full_matrix << std::endl;
    //printf("decoder_matrix: %d x %d\n", (int)decoder_matrix.rows(), (int)decoder_matrix.cols());
    //std::cout << decoder_matrix << std::endl;
    //printf("WILL SH COEFFICIENTS: %d x %d = %d\n", (int)sh.rows(), (int)sh.cols(), (int)sh.size());
    //std::cout << sh << std::endl;
    //printf("--END\n");

    // TODO(will): (opt?) perform ambisonic shelf-filtering

    auto sh_hrirs = vraudio::CreateShHrirsFromMatrix(
        sh, source_sampling_rate, graph_manager_->system_settings_.GetSampleRateHz(), &graph_manager_->resampler_);
    graph_manager_->ambisonic_binaural_decoder_node_[ambisonic_order]->user_decoder = new AmbisonicBinauralDecoder(
        *sh_hrirs, graph_manager_->system_settings_.GetFramesPerBuffer(), &graph_manager_->fft_manager_);

    // close sofa file
    mysofa_close(hrtf);

    return true;
};

bool ResonanceAudioApiImpl::EnableCustomSofa(bool enable_sofa, int ambisonic_order) {
    auto decoder_node = graph_manager_->ambisonic_binaural_decoder_node_[ambisonic_order];

    if (enable_sofa && decoder_node->user_decoder == nullptr)
        return false;

    if (enable_sofa)
        decoder_node->ambisonic_binaural_decoder_ = decoder_node->user_decoder;
    else
        decoder_node->ambisonic_binaural_decoder_ = decoder_node->builtin_decoder;

    return true;
};


}  // namespace vraudio
