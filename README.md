# waken
A ESP powered Smart watch which gives you a little zap to wake you up silently without disturbing anyone around you.

# why I built it ?
Ahhhh! I'm not crazy, I just sleep like a stone . So to get me up on time, I caught an idea why not make a watch which sits on my wrist and give me a little zap of 400V (\j) to wake me up . 

# Demo link
   https://youtu.be/53Z-3iZXITg?si=bSotyN73lvlZD2hz

# Zine

 <img width="540" height="828" alt="Frame 7" src="https://github.com/user-attachments/assets/55399966-3c21-4b40-9d7e-e50a8439b11e" />

 <img width="540" height="828" alt="Frame 8" src="https://github.com/user-attachments/assets/985dc4fe-2637-450e-a516-bfd46b1d95c8" />

 

# prototype 01

<img width="3264" height="2448" alt="IMG_3605" src="https://github.com/user-attachments/assets/2705a334-2e00-4828-8da5-d52d203bac75" />


<img width="3264" height="2448" alt="IMG_3602" src="https://github.com/user-attachments/assets/2d9aac57-94ac-4a26-861a-6ef5c21fc0ed" />


<img width="3264" height="2448" alt="IMG_3601" src="https://github.com/user-attachments/assets/1d4dde9c-8a23-4d5e-9558-52654832c525" />


<img width="3264" height="2448" alt="IMG_3595" src="https://github.com/user-attachments/assets/955a36b4-190c-4a4e-9c49-af1db9f85d6f" />


<img width="967" height="545" alt="WhatsApp Image 2026-08-12 at 00 28 32" src="https://github.com/user-attachments/assets/3194fef2-0171-4d89-9a11-b4bc241e5e4b" />

<img width="3072" height="4080" alt="WhatsApp Image 2026-08-12 at 00 47 58 (1)" src="https://github.com/user-attachments/assets/fb2b4b73-d5f2-42de-9f53-3ae5170d621a" />

 
 
# Shematics 
I've used EasyEDA for design designing this project

<img width="617" height="438" alt="Screenshot 2026-07-18 at 12 30 59 AM" src="https://github.com/user-attachments/assets/0d1f027f-dd7e-480b-9a1b-e85aeeb09c6a" />

 It has ESP32S3-WROON-1-N16R8 as the main brains , it has been built RTC and feature both Wi-Fi and Bluetooth with access to multiple GPIOs .

 <img width="512" height="464" alt="Screenshot 2026-07-19 at 11 32 55 PM" src="https://github.com/user-attachments/assets/81238b8e-f0a3-4e7f-8ea1-5ce6c15fb870" />


 For tracking motion and physical activities like step count, etc. I've used MPU6500 which features both I2C & SPI interface. I've used I2C interface for this project.
 
<img width="497" height="286" alt="Screenshot 2026-07-19 at 11 33 51 PM" src="https://github.com/user-attachments/assets/ce4e4a27-5df5-406d-9118-ddd10913c0b6" />


 For sensing temperature I used MCP9808 which also offers I2C interface so its easy to wire all of them in signle bus 

 <img width="398" height="194" alt="Screenshot 2026-07-19 at 11 35 23 PM" src="https://github.com/user-attachments/assets/705904fb-1d01-4058-bd3d-02ed57bf9b02" />


 The heart rate sensor is MAX30101 with I2C interface which offers optical pulse oximetry and heart-rate sensing .

 <img width="370" height="155" alt="Screenshot 2026-07-19 at 11 37 22 PM" src="https://github.com/user-attachments/assets/ab6728e3-98bb-4c35-9865-97661d215424" />


 To charge the lipo battery i have integrated a charging ic for 1s lipo and also integrated a load sharing circuit so that i can use teh device while charging .

 <img width="314" height="266" alt="Screenshot 2026-07-19 at 11 39 22 PM" src="https://github.com/user-attachments/assets/207dcafe-b23d-4d15-9512-79d9dd55ec4e" />


 To provide a constant supply of 3.3 V I am integrated a buck boost converter output of 3.3V @1.5A 
 
 <img width="278" height="166" alt="Screenshot 2026-07-19 at 11 41 27 PM" src="https://github.com/user-attachments/assets/923f7e85-1157-4605-9819-567f895527a7" />


 Now for that little zap i have used a simple fly back circuit with a couple inductor of scale 1:100 and IRLR7843 N chaneel mosfet to turn it on and off and a some massive 400v rated caps to store the build up with a series resistance of 150K ohm for safety .
 
 <img width="379" height="192" alt="Screenshot 2026-07-19 at 11 44 28 PM" src="https://github.com/user-attachments/assets/762af019-f09d-4c38-8c01-623bf5665022" />


For Inputs and outputs we have 2 push buttons and a side scroll wheel with oled and buzzers as our output 

<img width="435" height="312" alt="Screenshot 2026-07-19 at 11 46 19 PM" src="https://github.com/user-attachments/assets/b666de83-fc63-4d0f-a354-eb07930a76b8" />



# PCB 
 I have used 4 layer configuration

 
Top layer:- signals and some components

<img width="528" height="504" alt="Screenshot 2026-07-18 at 7 22 29 PM" src="https://github.com/user-attachments/assets/f7daee52-7085-4899-b205-fe2e8f192809" />


First inner layer:- ground

<img width="542" height="512" alt="Screenshot 2026-07-18 at 7 22 35 PM" src="https://github.com/user-attachments/assets/bcbf0bef-edd8-419a-9949-38a70cb4d801" />


Second inner layer:- VCC

<img width="521" height="494" alt="Screenshot 2026-07-18 at 7 22 40 PM" src="https://github.com/user-attachments/assets/e33a17b3-2d94-403e-b5b6-b7e1ffd92830" />


Bottom layer:- signal and all other components

<img width="533" height="499" alt="Screenshot 2026-07-18 at 7 22 23 PM" src="https://github.com/user-attachments/assets/b92644fa-98d8-4cb4-a602-5b144a6188bb" />


Important Note : I have not connected the ground of flyback with the rest of the circuit I will use a physical wire to connect when I receive a PCB.

<img width="2880" height="1226" alt="FRONT" src="https://github.com/user-attachments/assets/6ae3abfa-6d3b-4524-9396-fb3ad41b689c" />

<img width="2880" height="1226" alt="BACK" src="https://github.com/user-attachments/assets/2124871d-a5b3-4b2d-a756-130ff04696bd" />



# CAD 

I have used fusion 360 student edition for designing the 3D case for this watch

It is a very simple case on which the PCB is screwed using two screws, and the top part is tucked in and glued 
<img width="2880" height="1226" alt="IMG_1" src="https://github.com/user-attachments/assets/b9adce1e-bea2-48eb-9f9a-59bb44e31418" />


# Assembly

<img width="650" height="1044" alt="Top Case" src="https://github.com/user-attachments/assets/de451a46-ece4-4d54-b66b-4904f5cad4ef" />

The pcb is screwed in 


<img width="1166" height="828" alt="1 step" src="https://github.com/user-attachments/assets/0a972aa3-2669-46f8-a59d-307177170d88" />





# Firmware 
 I stated a very brief sample test code till now. When I will receive my PCB, then I will work on the rest of the firmware.







