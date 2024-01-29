package frc.robot;

public final class DeviceIds {
    public static final class Feeder {
        public static final int MotorId = 20;
        public static final int BeamBreakChannel = 0;
    }

    public static final class Intake {
        public static final int LeadMotorId = 19;
        public static final int FollowerMotorId = 18;
    }

    public static final class Shooter {
        public static final int LeftMotorId = 13;
        public static final int RightMotorId = 14;
        public static final int WristLeadMotorId = 15;
        public static final int WristFollowerMotorId = 16;
    }

    public static final class CANdle {
        public static final int CANdleId = 17;
    }

    public static final class Limelight {
        public static final String FrontTableName = "limelight-shooter";
        public static final String RearTableName = "limelight-intake";
    }
}
