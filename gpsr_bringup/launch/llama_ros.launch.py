from launch import LaunchDescription
from llama_bringup.utils import create_llama_launch


def generate_launch_description():

    return LaunchDescription([
        create_llama_launch(
            n_ctx=8192,
            n_batch=512,
            n_gpu_layers=29,
            n_threads=-1,
            n_predict=2000,

            model_repo="cstr/Spaetzle-v60-7b-Q4_0-GGUF",
            model_filename="Spaetzle-v60-7b_Q4_0.gguf",
            
            # model_repo="bartowski/Qwen2.5-7B-Instruct-GGUF",
            # model_filename="Qwen2.5-7B-Instruct-Q6_K_L.gguf",
            
            # model_repo="lmstudio-community/Llama-3.2-1B-Instruct-GGUF",
            # model_filename="Llama-3.2-1B-Instruct-Q8_0.gguf",
            
            # model_repo="Qwen/Qwen2.5-0.5B-Instruct-GGUF",
            # model_filename="qwen2.5-0.5b-instruct-fp16.gguf",
            
            # model_repo="bartowski/Llama-3.2-3B-Instruct-GGUF",
            # model_filename="Llama-3.2-3B-Instruct-Q6_K.gguf",

            stopping_words=[f"eot_id", f"<|im_end|>", "<|end|>", "<|im_start|>", "<|endoftext|>"
                            # , "</s>", "\n\n\n\n"
                            ],

            # lora_adapters= [
            #     # {
            #     #     "path": "/home/robotica/tools/unsloth/loras/Qwen2.5-0.5B-Instruct-GPSR-BF16-LoRA.gguf",
            #     #     "scale": 1.0
            #     # },
            #     # # {
            #     # #     "path": "/home/robotica/tools/unsloth/loras/Llama-3.2-3B-Instruct-GPSR-BF16-LoRA.gguf",
            #     # #     "scale": 1.0
            #     # # },
            # ],

            debug=True
        )
    ])
