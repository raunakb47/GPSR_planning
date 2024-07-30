from launch import LaunchDescription
from llama_bringup.utils import create_llama_launch


def generate_launch_description():

    return LaunchDescription([
        create_llama_launch(
            n_ctx=8192,
            n_batch=256,
            n_gpu_layers=33,
            n_threads=1,
            n_predict=-1,

            model_repo="cstr/Spaetzle-v60-7b-Q4_0-GGUF",
            model_filename="Spaetzle-v60-7b_Q4_0.gguf",

            # model_repo="lmstudio-community/Meta-Llama-3.1-8B-Instruct-GGUF",
            # model_filename="Meta-Llama-3.1-8B-Instruct-Q4_K_M.gguf",

            # model_repo="internlm/internlm2_5-7b-chat-gguf",
            # model_filename="internlm2_5-7b-chat-q4_k_m.gguf",

            # model_repo="TheBloke/Mistral-7B-Instruct-v0.2-GGUF",
            # model_filename="mistral-7b-instruct-v0.2.Q4_K_M.gguf",

            stopping_words=[f"eot_id", f"im_end", "</s>", "\n\n\n\n"],

            debug=True
        )
    ])
