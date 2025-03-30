import os
import pyaudio
import wave
from google import genai
import time
import requests
import json
from playsound import playsound
import numpy as np
from gtts import gTTS
import config as cf


cf.cf_destination = "none"


class virtual_assistance:
    def __init__(self):
        print("Bắt đầu trợ lý ảo.")
        self.count_call = 0

    def get_record(self):
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        CHUNK = 1024
        OUTPUT_FILENAME = "recorded_audio_1.wav"
        SILENCE_THRESHOLD = 4000  # Ngưỡng năng lượng để phát hiện khoảng lặng
        SILENCE_DURATION = 3  # Số giây im lặng liên tiếp để dừng ghi âm

        audio = pyaudio.PyAudio()
        stream = audio.open(format=FORMAT, channels=CHANNELS,
                            rate=RATE, input=True,
                            frames_per_buffer=CHUNK)

        print("🔴 Đang ghi âm... (Tự động dừng khi phát hiện khoảng lặng)")

        frames = []
        silence_start = None  # Thời điểm bắt đầu khoảng lặng

        while True:
            data = stream.read(CHUNK)
            # print("--------------1---------------")

            frames.append(data)
            # print("--------------2---------------")

            # Chuyển dữ liệu âm thanh thành mảng numpy
            audio_data = np.frombuffer(data, dtype=np.int16)
            energy = np.abs(audio_data).mean()  # Tính năng lượng trung bình

            # print(energy)

            # Kiểm tra nếu âm thanh nhỏ hơn ngưỡng khoảng lặng
            if energy < SILENCE_THRESHOLD:
                if silence_start is None:
                    silence_start = time.time()  # Bắt đầu đếm thời gian khoảng lặng
                elif time.time() - silence_start >= SILENCE_DURATION:
                    print("🟢 Khoảng lặng dài, dừng ghi âm!")
                    break
            else:
                silence_start = None  # Reset nếu có âm thanh trở lại

        print("🟢 Ghi âm hoàn tất!")

        stream.stop_stream()
        stream.close()
        audio.terminate()

        wf = wave.open(OUTPUT_FILENAME, "wb")
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(audio.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()

        print(f"✅ File đã được lưu: {OUTPUT_FILENAME}")


    def understanding_record(self):
        # api_key: AIzaSyCTqQM557q_5iUS-RQs661124KDC_wGryM
        client = genai.Client(
        api_key="AIzaSyAIptARWvsfvfWfubmwI0eBMrBZm2t34oc",
        )

        myfile = client.files.upload(file='recorded_audio_1.wav')

        prompt =  """ Bạn là trợ lý ảo trên xe tự hành của phòng thí nghiệm hệ thống thông minh trường đại học sư phạm kỹ thuật thành phố Hồ Chí Minh,  
                    lấy nội dung đoạn hôi thoại trên, người dùng có 3 nhu cầu hãy phân loại thành 3 loại bên dưới: 
                    - 1 là họ yêu cầu đến các khu c, khu d, cổng trường, tòa nhà trung tâm, tòa việt đức, xưởng gỗ thì trả về đoạn text tương ứng khu_c, khu_d, trung_tam, viet_duc, go
                    - 2 là câu hỏi kiến thức về trường đại học sư phạm kỹ thuật, một người nào đó thì hãy cung cấp thông tin chính xác và nghiêm túc
                    - 3 là họ muốn yêu cầu khác hãy phản hồi một cách hài hước.

                    Đây là thông tin của bạn:
                    - tên: trợ lý ảo trên xe tự hành của phòng thí nghiệm hệ thống thông minh trường đại học sư phạm kỹ thuật thành phố Hồ Chí Minh
                    - bạn có thể chở mọi người đến các địa điểm sau: khu c, khu d, cổng trường, tòa nhà trung tâm, tòa việt đức, xưởng gỗ của trường đại học sư phạm kỹ thuật thành phố Hồ Chí Minh.
                    - nếu có câu hỏi nào khác về bạn hãy trả lời một cách hài hước nhưng vẫn lịch sự.

                    một số kiến thức khác:
                    - Giáo sư Lê Mỹ Hà là một giảng viên, nhà nghiên cứu uy tín của trường Đại học Sư phạm Kỹ thuật TP.HCM.
                    - Thầy Lê Hiếu Giang là hiệu trưởng Đại học Sư phạm Kỹ thuật TP.HCM.

                    lưu ý khi phản hồi:
                    - không kèm các icon trong nội dung.
                    - phản hồi xúc tích, không lặp toàn bộ prompt này.
                    - nếu không nhận được yêu cầu hãy giới thiệu bản thân và bạn có thể làm gì.
                    """

        response = client.models.generate_content(
        model='gemini-2.0-flash',
        contents=[prompt, myfile]
        )

        return response.text
    
    def get_speech(self, text):

        # print(text)
        is_run = False

        print(f"Nhận được text: {repr(text)}")

        destination = {
            'khu_c': "Tôi sẽ đưa bạn đến Khu c nhé.",
            'khu_d': "Tôi sẽ đưa bạn đến Khu d nhé.",
            'trung_tam': "Tôi sẽ đưa bạn đến tòa nhà Trung tâm nhé.",
            'viet_duc': "Tôi sẽ đưa bạn đến tòa Việt Đức nhé.",
            'go': "Tôi sẽ đưa bạn đến tòa xưởng gỗ nhé."
        }

        if text.strip().lower() in destination:
            text_respond = destination[text.strip().lower()]
            cf.cf_destination = text.strip().lower()
            is_run = True
        else:
            text_respond = text


        print("phản hồi: ", text_respond)


        tts = gTTS(text = text_respond, lang="vi")

        tts.save("./Assistance_Astar/output.mp3")
        time.sleep(0.1)
        playsound("./Assistance_Astar/output.mp3")
        os.remove("./Assistance_Astar/output.mp3")

        return is_run



        

        # url = 'https://api.fpt.ai/hmi/tts/v5'
        # payload = text
        # headers = {
        #     'api-key': 'K5jOdAj46wLFqOHkgDwROHE7AFs7ZlCx',
        #     'speed': '',
        #     'voice': 'banmai'
        # }

        # response = requests.post(url, data=payload.encode('utf-8'), headers=headers)
        # response_data = json.loads(response.text)

        # audio_url = response_data.get("async")

        # if audio_url:
        #     print("Tải file từ:", audio_url)

        #     # Chờ tối đa 10 giây để file sẵn sàng
        #     for _ in range(10):  
        #         audio_response = requests.get(audio_url)
        #         if audio_response.status_code == 200:
        #             with open("output.mp3", "wb") as file:
        #                 file.write(audio_response.content)
        #             print("Tải xuống thành công: output.mp3")
        #             break
        #         else:
        #             print("File chưa sẵn sàng, thử lại...")
        #             time.sleep(1)  # Chờ 1 giây rồi thử lại
        #     else:
        #         print("Lỗi khi tải file âm thanh!")
        #         return

        #     time.sleep(0.2)
        #     playsound("output.mp3")
        #     os.remove("output.mp3")
        # else:
        #     print("Không tìm thấy URL âm thanh!")

        
    
    def run(self):

        self.get_record()
        # time.sleep(0.2)
        text = self.understanding_record()
        # time.sleep(0.2)
        # print(text)
        is_run = self.get_speech(text)

        return is_run

        # print(text)

def run_assistance():
    islab_assistance = virtual_assistance()

    while True:

        is_run = islab_assistance.run()

        print(is_run)

        if is_run == True:
            break

if __name__ == "__main__":
    run_assistance()
