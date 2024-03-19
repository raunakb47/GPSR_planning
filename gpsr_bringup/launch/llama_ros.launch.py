from launch import LaunchDescription
from llama_bringup.utils import create_llama_launch


def generate_launch_description():

    return LaunchDescription([
        create_llama_launch(
            n_ctx=4096,
            n_batch=256,
            n_gpu_layers=33,
            n_threads=4,
            n_predict=-1,

            model_repo="TheBloke/Marcoroni-7B-v3-GGUF",
            model_filename="marcoroni-7b-v3.Q4_K_M.gguf",
            
            stop="\n\n\n\n",

            debug=True
        )
    ])