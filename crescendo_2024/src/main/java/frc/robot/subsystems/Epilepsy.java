package frc.robot.subsystems;

import frc.robot.Constants.EpilepticSeizureOfProperty;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Epilepsy extends SubsystemBase{
    AddressableLED led = new AddressableLED(EpilepticSeizureOfProperty.portNumber);
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(EpilepticSeizureOfProperty.ledLength);
    public boolean uncool = false;
    public boolean rainbowMode;
    int rainbowFirstPixelHue;

    public Epilepsy(){
        try{
            led = new AddressableLED(EpilepticSeizureOfProperty.portNumber);
        }catch(Exception e){
            System.out.println("UNCOOLNESS DETECTED - LEDS WRONG PORT");
            uncool = true;
        };
        if (!uncool){
            led.setLength(ledBuffer.getLength());
            reinforced();
            led.setData(ledBuffer);
            led.start();
        }
    }

    @Override
    public void periodic(){
        if(!uncool){
            if(rainbowMode)
                gameing();
            led.setData(ledBuffer);
        }
    }

    public void colorChooser(int c){
        if(!uncool){
            switch(c){
                case 0:
                    reinforced(); //set purple - should be the default
                    rainbowMode = false;
                    break;
                case 1:
                    betterAllianceColor(); //set blue - objectively better alliance color 
                    rainbowMode = false;
                    break;
                case 2: 
                    worseAllianceColor(); //set red - worse alliance color
                    rainbowMode = false;
                    break;
                case 3:
                    rainbowMode = true; //set rainbow - gameing mode because our auto is just like that
                    break;
                case 4: 
                    minuteMaidLimeade(); //set pulsing green - ready to shoot
                    rainbowMode = false;
                    break;
                case 5:
                    slightOopsie(); //set pulsing red - now everyone knows it messed up
                    rainbowMode = false;
                    break;
                case 6:
                    blastWhiteGirlMusic(); //set pulsing pink - for a certain special someone
                    rainbowMode = false;
                    break;
            }
        }
    }

    public void reinforced(){
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 140, 25, 175);
        }
    }

    public void betterAllianceColor(){
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 20, 80, 210);
        }
    }

    public void worseAllianceColor(){
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 230, 20, 0);
        }
    }

    public void gameing(){
        // Increase silliness
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
          }
          // Increase by to make the rainbow "move"
          rainbowFirstPixelHue += 3;
          // Check bounds
          rainbowFirstPixelHue %= 180;
    }

    public void minuteMaidLimeade(){
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 15, 200, 65);
        }
    }

    public void slightOopsie(){
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 180, 25, 30);
        }
    }

    public void blastWhiteGirlMusic(){
        // It's purrfect for a certain Ian
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 255, 100, 210);
        }
    }
}
