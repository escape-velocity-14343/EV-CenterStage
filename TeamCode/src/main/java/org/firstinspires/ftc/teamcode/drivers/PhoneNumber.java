package org.firstinspires.ftc.teamcode.drivers;

import android.util.Log;

public class PhoneNumber {
    long number = 0;
    String name = "owen";
    public PhoneNumber(long number, String name) {
        this.number = number;
        this.name = name;
    }
    public void call() {
        Log.println(Log.ERROR, name, "Hello this is " + name + "from microsoft tech support how may i help you today?");
    }
}
