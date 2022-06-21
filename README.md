# Heart-Rate-Monitor-for-STM32F405RG-MCU

Hello! What you're looking at here is all the files I created and modified. With the exception of some files made by the instructors for the students to use.

The system requires my design of a heart rate monitor board, I would be happy to share those files with those who are interetested and they6 can reach out to me
vie a email link on my website www.michaelwreid.ca. 

For the most part this code is quite simple, asign all the ARMS pins to what I want and what my project mates also want. Then take the signal in from my prototype 
which is a pulse. Or rather its the missing light it reads from your finger filling with blood each pulse. That singla is cleaned up and amplified so a digital to analog converter can safely carry the singla to the MCU.

After the singal is in the ARM the code I wrote in HRM_analysis takes the single, and averages it. Origonally the averageing could have been changed based on a persons pulse so it could display fater changes. But the average is then run there the LCD display i2C function and displays to a Hitatchi screen because it was the cheapest. If you know what Hitatchi is, no you dont.

Thats really all there is to this project, covid kind of killed our ability to add the full functionality we wanted. When I say we I mean my group mates and I. the full project was a Bio metric feedback monitor, a lie detector basiaclly. THe others tracked skin resistance for sweat and the other tracked breathiing rate. I helped them with their coding but thats for them to share not me!.

Anyways, feel free to reach out if you'd like further clarification on anything, don't judge my spelling, I was raised in a world with spell check haha.

  -Michael
