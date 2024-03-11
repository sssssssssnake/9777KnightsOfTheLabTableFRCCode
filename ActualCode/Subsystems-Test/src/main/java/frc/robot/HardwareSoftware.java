package frc.robot;

public class HardwareSoftware {

    //Neo Brushless Motor
    public static int mDeliveryRight = 6;
    public static int mDeliveryLeft = 3;

    //Neo Brushless Motor
    public static int mIntake = 5;

    //Neo Brushless Motor
    public static int mHangRight = 4;
    public static int mHangLeft = 1;


    public DeliverySubsystem delivery;
    public IntakeSubsystem intake;
    public HangSubsystem hang;

    HardwareSoftware(){


    }

    public void init(){
        delivery = new DeliverySubsystem(mDeliveryRight, mDeliveryLeft);
        intake = new IntakeSubsystem(mIntake);
        hang = new HangSubsystem(mHangRight, mHangLeft);
    }

}
