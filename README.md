# Tiny Reinforcement Learning (TinyRL) - Pendulum Demo

This repo demonstrates training a reinforcement learning (RL) agent on a real, hardware inverted pendulum kit: [STEVAL-EDUKIT01](https://www.digikey.com/en/products/detail/stmicroelectronics/STEVAL-EDUKIT01/11696333). 

 1. Load the *pendulum-controller/pendulum-controller-esp32* firmware onto the ESP32 to act as an interface for the Python script
 2. Run through the cells in *pendulum-rl.ipynb* to perform hyperparameter optimization, agent training, and model extraction
 3. Convert .onnx model to Arduino library using e.g. Edge Impulse
 4. Load *embedded-inference/esp32-inference.ino* onto the ESP32 to have the agent run locally on the microcontroller

## License

All code, unless otherwise noted, is licensed under the [Zero-Clause BSD (0BSD) license](https://opensource.org/license/0bsd/).

Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.