package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import java.util.ArrayList;

public class Button {

    private boolean lastPressed = false;

    public Button() {}

    /**
     * please run at start of loops
     * note: if down on first call, will return a true onPress value
     * @param currPressed
     * @return
     */
    public Data update(boolean currPressed) {
        Data res = new Data(
                currPressed && !lastPressed,
                currPressed,
                !currPressed && lastPressed
        );

        this.lastPressed = currPressed;

        return res;
    }

    public static class Data {
        public final boolean onPress;
        public final boolean onHold;
        public final boolean onUnpress;

        public Data(boolean onPress, boolean onHold, boolean onUnpress) {
            this.onPress = onPress;
            this.onHold = onHold;
            this.onUnpress = onUnpress;
        }

        @NonNull
        @Override
        public String toString() {
            ArrayList<String> propertiesTrue = new ArrayList<>();

            if (onPress) propertiesTrue.add("onPress");
            if (onHold) propertiesTrue.add("onHold");
            if (onUnpress) propertiesTrue.add("onUnpress");

            return String.join(" ", propertiesTrue);
        }
    }

}
