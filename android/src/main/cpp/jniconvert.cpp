#include <jni.h>
#include <string>

#include "thneedmodel.h"

ThneedModel::ThneedModel(const std::string path, float *_output, size_t _output_size, int runtime, bool luse_tf8, cl_context context) {
    thneed = new Thneed(true, context);
    thneed->load(path.c_str());
    thneed->clexec();

    recorded = false;
    output = _output;
}

void* ThneedModel::getCLBuffer(const std::string name) {
    int index = -1;
    for (int i = 0; i < inputs.size(); i++) {
        if (name == inputs[i]->name) {
            index = i;
            break;
        }
    }

    if (thneed->input_clmem.size() >= inputs.size()) {
        return &thneed->input_clmem[inputs.size() - index - 1];
    } else {
        return nullptr;
    }
}

void ThneedModel::execute() {
    if (!recorded) {
        thneed->record = true;
        float *input_buffers[inputs.size()];
        for (int i = 0; i < inputs.size(); i++) {
            input_buffers[inputs.size() - i - 1] = inputs[i]->buffer;
        }

        thneed->copy_inputs(input_buffers);
        thneed->clexec();
        thneed->copy_output(output);
        thneed->stop();

        recorded = true;
    } else {
        float *input_buffers[inputs.size()];
        for (int i = 0; i < inputs.size(); i++) {
            input_buffers[inputs.size() - i - 1] = inputs[i]->buffer;
        }
        thneed->execute(input_buffers, output);
    }
}

std::string *pathString;
jfloat* outputs;
jint output_len;
ThneedModel *thneed;
jboolean inputsSet = false;

extern "C" {

    void JNICALL Java_ai_flow_android_vision_THNEEDModelRunner_createStdString(JNIEnv *env, jclass clazz, jstring javaString) {
        // Convert Java string to C++ string
        const char *cString = env->GetStringUTFChars(javaString, 0);
        pathString = new std::string(cString);

        // Release the C string
        env->ReleaseStringUTFChars(javaString, cString);
    }

    void JNICALL Java_ai_flow_android_vision_THNEEDModelRunner_getArray(JNIEnv *env, jobject obj, jint size) {
        // Allocate a float array of the given size
        outputs = new jfloat[size];
        output_len = size;
    }

    void JNICALL Java_ai_flow_android_vision_THNEEDModelRunner_initThneed(JNIEnv *env, jobject obj) {
        cl_int err;
        cl_uint numPlatforms;
        err = clGetPlatformIDs(0, NULL, &numPlatforms);
        std::vector<cl_platform_id> platforms(numPlatforms);
        err = clGetPlatformIDs(numPlatforms, platforms.data(), NULL);
        cl_uint numDevices;
        err = clGetDeviceIDs(platforms[0], CL_DEVICE_TYPE_GPU, 0, NULL, &numDevices); // Change to CPU if no GPU is available
        std::vector<cl_device_id> devices(numDevices);
        err = clGetDeviceIDs(platforms[0], CL_DEVICE_TYPE_GPU, numDevices, devices.data(), NULL); // Change to CPU if no GPU is available
        cl_context context = clCreateContext(NULL, numDevices, devices.data(), NULL, NULL, &err);
        thneed = new ThneedModel(*pathString, outputs, output_len, 0, false, context);
    }

    JNIEXPORT jfloatArray JNICALL Java_ai_flow_android_vision_THNEEDModelRunner_executeModel(JNIEnv *env, jobject obj,
                                                               jfloatArray input_imgs,
                                                               jfloatArray big_input_imgs,
                                                               jfloatArray features_buffer,
                                                               jfloatArray desire,
                                                               jfloatArray traffic_convention,
                                                               jfloatArray nav_features,
                                                               jfloatArray nav_instructions) {
        // get sizes
        jsize input_imgs_len = env->GetArrayLength(input_imgs);
        jsize big_input_imgs_len = env->GetArrayLength(big_input_imgs);
        jsize features_buffer_len = env->GetArrayLength(features_buffer);
        jsize desire_len = env->GetArrayLength(desire);
        jsize traffic_convention_len = env->GetArrayLength(traffic_convention);
        jsize nav_features_len = env->GetArrayLength(nav_features);
        jsize nav_instructions_len = env->GetArrayLength(nav_instructions);

        // buffers
        jfloat *input_imgs_buf = env->GetFloatArrayElements(input_imgs, 0);
        jfloat *big_input_imgs_buf = env->GetFloatArrayElements(big_input_imgs, 0);
        jfloat *features_buffer_buf = env->GetFloatArrayElements(features_buffer, 0);
        jfloat *desire_buf = env->GetFloatArrayElements(desire, 0);
        jfloat *traffic_convention_buf = env->GetFloatArrayElements(traffic_convention, 0);
        jfloat *nav_features_buf = env->GetFloatArrayElements(nav_features, 0);
        jfloat *nav_instructions_buf = env->GetFloatArrayElements(nav_instructions, 0);

        if (inputsSet) {
            thneed->setInputBuffer("input_imgs", input_imgs_buf, input_imgs_len);
            thneed->setInputBuffer("big_input_imgs", big_input_imgs_buf, big_input_imgs_len);
            thneed->setInputBuffer("features_buffer", features_buffer_buf, features_buffer_len);
            thneed->setInputBuffer("desire", desire_buf, desire_len);
            thneed->setInputBuffer("traffic_convention", traffic_convention_buf, traffic_convention_len);
            thneed->setInputBuffer("nav_features", nav_features_buf, nav_features_len);
            thneed->setInputBuffer("nav_instructions", nav_instructions_buf, nav_instructions_len);
        } else {
            thneed->addInput("input_imgs", input_imgs_buf, input_imgs_len);
            thneed->addInput("big_input_imgs", big_input_imgs_buf, big_input_imgs_len);
            thneed->addInput("features_buffer", features_buffer_buf, features_buffer_len);
            thneed->addInput("desire", desire_buf, desire_len);
            thneed->addInput("traffic_convention", traffic_convention_buf, traffic_convention_len);
            thneed->addInput("nav_features", nav_features_buf, nav_features_len);
            thneed->addInput("nav_instructions", nav_instructions_buf, nav_instructions_len);
            inputsSet = true;
        }

        // ok execute model
        thneed->execute();

        // When done, release the memory
        env->ReleaseFloatArrayElements(input_imgs, input_imgs_buf, 0);
        env->ReleaseFloatArrayElements(big_input_imgs, big_input_imgs_buf, 0);
        env->ReleaseFloatArrayElements(features_buffer, features_buffer_buf, 0);
        env->ReleaseFloatArrayElements(desire, desire_buf, 0);
        env->ReleaseFloatArrayElements(traffic_convention, traffic_convention_buf, 0);
        env->ReleaseFloatArrayElements(nav_features, nav_features_buf, 0);
        env->ReleaseFloatArrayElements(nav_instructions, nav_instructions_buf, 0);

        // get the outputs
        jfloatArray result = env->NewFloatArray(output_len);
        env->SetFloatArrayRegion(result, 0, output_len, outputs);

        return result;
    }

} // extern "C"
