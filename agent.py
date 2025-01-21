import streamlit as st
from typing import List
import socket
import speech_recognition as sr
import os
import cv2
import sys
from langchain_google_genai import ChatGoogleGenerativeAI
import os
if "GOOGLE_API_KEY" not in os.environ:
    os.environ["GOOGLE_API_KEY"] = "GOOGLE_API_KEY"
import time
from gtts import gTTS
from IPython.display import Audio
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain.prompts import (
    PromptTemplate,
    SystemMessagePromptTemplate,
    HumanMessagePromptTemplate,
    ChatPromptTemplate,
)
from langchain_core.output_parsers import StrOutputParser
from langchain.schema.runnable import RunnablePassthrough
from langchain.agents import (
    create_openai_functions_agent,
    Tool,
    AgentExecutor,
)
from langchain import hub
from FruitDetector import FruitDetector

class FruitOrderAgent:
    def __init__(self):
        self.detector = FruitDetector()
        self.current_order = []
        self.llm = ChatGoogleGenerativeAI(model="gemini-2.0-flash-exp",
                                          temperature=0.3, convert_system_message_to_human=True)
        self.tools = self._initialize_tools()
        self.agent_executor = self._initialize_agent()

    def _initialize_tools(self):
        return [
            Tool(
                name="OrderOrange",
                func=self.order_orange,
                description="استخدم هذه الأداة لطلب برتقالة. قم بتنفيذها ببساطة دون إدخال أي بيانات.",
            ),
            Tool(
                name="OrderBanana",
                func=self.order_banana,
                description="استخدم هذه الأداة لطلب موزة. قم بتنفيذها ببساطة دون إدخال أي بيانات.",
            ),
            Tool(
                name="EndOrder",
                func=self.end_order,
                description="استخدم هذه الأداة لإنهاء طلب الفاكهة. ستوفر هذه الأداة ملخصًا للطلب\nوتفرغ الطلب الحالي. استخدمها عندما ينتهي المستخدم من إضافة العناصر.",
            ),
        ]

    def _initialize_agent(self):
        agent_prompt = hub.pull("hwchase17/openai-functions-agent")
        agent_prompt.messages.remove(SystemMessagePromptTemplate(prompt=PromptTemplate(input_variables=[], template='You are a helpful assistant')))
        agent_prompt.messages.insert(0, SystemMessagePromptTemplate(prompt=PromptTemplate(input_variables=[], template="""أنت بائع فواكه تتحدث باللهجة المصرية. تساعد العملاء في شراء الفواكه، وإنهاء الطلبات.
        يجب أن تجيب بطريقة ودودة وبأسلوب حواري. هدفك هو تقديم خدمة ممتازة والحفاظ على ترتيب الطلبات.

        لديك الأدوات التالية لتنفيذ الطلبات، ويجب عليك استخدامها فقط:

        1. **طلب برتقالة**:
           - الوصف: استخدم هذه الأداة لإضافة برتقالة إلى الطلب الحالي. قم بتنفيذها دون الحاجة إلى إدخال أي بيانات إضافية.

        2. **طلب موزة**:
           - الوصف: استخدم هذه الأداة لإضافة موزة إلى الطلب الحالي. قم بتنفيذها دون الحاجة إلى إدخال أي بيانات إضافية.

        3. **إنهاء الطلب**:
           - الوصف: استخدم هذه الأداة لإنهاء طلب الفاكهة. ستعرض هذه الأداة ملخصًا للطلب الذي تم تجميعه وستقوم بإعادة تعيين الطلب.

        تذكر أنك ملزم باستخدام الأدوات فقط عند تلبية طلبات العملاء.""")))
        return AgentExecutor(
            agent=create_openai_functions_agent(llm=self.llm, prompt=agent_prompt, tools=self.tools),
            tools=self.tools,
            return_intermediate_steps=True,
            verbose=True,
        )

    def capture_image(self):
        cap = cv2.VideoCapture(1)
        if not cap.isOpened():
            st.error("Error: Could not open camera.")
            return None

        ret, frame = cap.read()
        cap.release()
        cv2.destroyAllWindows()

        if ret:
            image_path = "temp_image.jpg"
            cv2.imwrite(image_path, frame)
            return image_path
        else:
            st.error("Error: Could not capture image.")
            return None

    def send_to_server(self, message):
        HOST = '192.168.x.x'  # Replace with Raspberry Pi's IP address
        PORT = 65432

        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            client_socket.connect((HOST, PORT))
            client_socket.sendall(message.encode())
            data = client_socket.recv(1024).decode()
            st.write(f"Received from server: {data}")
        finally:
            client_socket.close()

    def order_orange(self, _=None):
        image_path = self.capture_image()
        if not image_path:
            return "فشل التقاط الصورة. حاول مرة أخرى."

        results = self.detector.run_inference(image_path=image_path)
        os.remove(image_path)

        for result in results:
            closest_orange = self.detector.detect_orange(result.boxes)
            if closest_orange:
                st.write("Closest and most accurate Orange:", closest_orange[0])
                self.current_order.append("برتقالة")
                #self.send_to_server(str(closest_orange[0]))
                return "تمت إضافة برتقالة إلى طلبك."
            else:
                return "لا يوجد برتقال الآن. حاول مرة أخرى لاحقًا."

    def order_banana(self, _=None):
        image_path = self.capture_image()
        if not image_path:
            return "فشل التقاط الصورة. حاول مرة أخرى."

        results = self.detector.run_inference(image_path=image_path)
        os.remove(image_path)

        for result in results:
            closest_banana = self.detector.detect_banana(result.boxes)
            if closest_banana:
                st.write("Closest and most accurate Banana:", closest_banana[0])
                self.current_order.append("موزة")
                #self.send_to_server(str(closest_banana[0]))
                return "تمت إضافة موزة إلى طلبك."
            else:
                return "لا يوجد موز الآن. حاول مرة أخرى لاحقًا."

    def end_order(self, _=None):
        if self.current_order:
            order_summary = f"لقد طلبت: {', '.join(self.current_order)}. شكرًا على طلبك!"
            self.current_order.clear()
            st.success(order_summary)
            sys.exit()
            return order_summary
        else:
            return "لا توجد عناصر في طلبك. يرجى إضافة فواكه إلى طلبك أولاً."

    def hp(self):
        stop_hp = False
        while not stop_hp:
            st.write("Waiting for user input...")
            audio_text = self.capture_audio_to_text()
            st.write(f"User said: {audio_text}")
            response = self.agent_executor.invoke({"input": audio_text})
            st.write(f"Agent action: {response.get('action')}")
            self.generate_audio(response["output"], "response")
            st.write(f"Agent response: {response['output']}")
            time.sleep(20)

    def capture_audio_to_text(self, timeout=15, phrase_time_limit=1000, language="ar-EG"):
        recognizer = sr.Recognizer()
        with sr.Microphone() as source:
            st.info("Adjusting for ambient noise... Please wait.")
            recognizer.adjust_for_ambient_noise(source, duration=1)
            st.info(f"Listening... (You have {phrase_time_limit} seconds to speak)")
            try:
                audio = recognizer.listen(source, timeout=timeout, phrase_time_limit=phrase_time_limit)
                st.info("Processing audio...")
                return recognizer.recognize_google(audio, language=language)
            except sr.WaitTimeoutError:
                return "الصوت مش واضح."
            except sr.UnknownValueError:
                return "الصوت مش واضح"
            except sr.RequestError as e:
                return f"API request error: {e}"

    def generate_audio(self, text, file_name):
        tts = gTTS(text, lang='ar')
        tts.save(f'{file_name}.mp3')
        audio_path = f"{file_name}.mp3"
        st.audio(audio_path, format="audio/mp3", start_time=0)

# Example Usage in Streamlit
agent = FruitOrderAgent()
st.title("Fruit Order Agent")
st.write("تطبيق يساعدك في طلب الفواكه وإتمام طلباتك.")

if st.button("ابدأ طلب الفواكه"):
    agent.hp()


