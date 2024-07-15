from launch import LaunchDescription
from llama_bringup.utils import create_llama_launch


def generate_launch_description():

    return LaunchDescription([
        create_llama_launch(
            n_ctx=4096,
            n_batch=256,
            n_gpu_layers=30,
            n_threads=4,
            n_predict=-1,

            model_repo="cstr/Spaetzle-v60-7b-Q4_0-GGUF",
            model_filename="Spaetzle-v60-7b_Q4_0.gguf",

            stopping_words=["\n\n\n\n"],

            debug=True
        )
    ])
