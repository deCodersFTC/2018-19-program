package com.example.sharad.threadedapp;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.*;
import android.os.Handler;

public class MainActivity extends AppCompatActivity {

    long startTime;
    Thread task;
    TextView mtvtimer;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        startTimer();
    }

    boolean stop = false;

    public String getTimeString(long time) {
        long millis = time - startTime;
        long secs = millis / 1000 % 60; // seconds, 0 - 59
        long mins = millis / 1000 / 60 % 60; // total seconds / 60, 0 - 59
        long hours = millis / 1000 / 60 / 60; // total seconds / 3600, 0 - limitless

        return String.format("%02d:%02d:%02d", hours, mins, secs);
    }

    public void startTimer() {
        System.out.println("************* In startTimer *************");
        startTime = System.currentTimeMillis();
        task = new Thread() {
            @Override
            public void run() {
                System.out.println("IN task.......");
                while (!stop) {
                    String timeString = getTimeString(System.currentTimeMillis());
                    System.out.println("AAAAAA " + timeString);
                    try {
                        Thread.sleep(1000); // robot work instead
                    } catch (Exception e) {
                    }
                }
            }
        };

        System.out.println("************* about to start task *************");
        System.out.println("BBBBBB " +
                getTimeString(System.currentTimeMillis()));
        task.start();
        while (!stop) {
            String timeString = getTimeString(System.currentTimeMillis());
            System.out.println("BBBBB " + timeString);
            long millis = System.currentTimeMillis() - startTime;
            long secs = millis / 1000 % 60; // seconds, 0 - 59
            long mins = millis / 1000 / 60 % 60; // total seconds / 60, 0 - 59
            long hours = millis / 1000 / 60 / 60; // total seconds / 3600, 0 - limitless
            if (secs >= 30) {
                System.out.println("Setting stop");
                stop = true;
            } else {
                try {
                    Thread.sleep(3000);
                } catch (Exception e) {
                }
            }

        }
    }
}
