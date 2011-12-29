package com.teamhacksung.volumehack.receiver;

import android.content.Context;
import android.content.BroadcastReceiver;
import android.content.Intent;
import android.content.IntentFilter;
import java.io.IOException;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.DataOutputStream;
import java.io.DataInputStream;
import android.util.Log;

import com.teamhacksung.volumehack.service.HeadsetService;

public class HeadsetPlugReceiver extends BroadcastReceiver {

	private static final String TAG = "VolumeHack";
    
    public void onReceive(Context context, Intent intent) { 
				
        if (intent.getExtras().getInt("state") == 0) { 
            /* HEADSET IS OR HAS BEEN DISCONNECTED */ 
            try {
				Thread.sleep(1000);
				Log.i(TAG, "Headset disconnected. Unapplying VolumeHack.");
				
				String [] commands = { "/system/bin/alsa_amixer sset 'DAC' 75", 
									  "/system/bin/alsa_amixer sset 'Headphone' 0", 
									  "/system/bin/alsa_amixer sset 'HP Gain' 0" };
				
				Process process = Runtime.getRuntime().exec("su");
				DataOutputStream os = new DataOutputStream(process.getOutputStream());
				DataInputStream osRes = new DataInputStream(process.getInputStream());
				for (String single : commands) {
					os.writeBytes(single + "\n");
					os.flush();
					Log.i(TAG, osRes.readLine());
				}
				os.writeBytes("exit\n");
				os.flush();
				process.waitFor();
				
			} catch (IOException e) {
				throw new RuntimeException(e);
			} catch (InterruptedException e) {
				throw new RuntimeException(e);
			}
			
        }else{ 
            /* HEADSET IS OR HAS BEEN CONNECTED */ 
            try {
				Thread.sleep(1000);
				Log.i(TAG, "Headset connected. Applying VolumeHack.");
				
				String [] commands = { "/system/bin/alsa_amixer sset 'DAC' 90", 
									  "/system/bin/alsa_amixer sset 'Headphone' 31", 
									  "/system/bin/alsa_amixer sset 'HP Gain' 3" };
				
				Process process = Runtime.getRuntime().exec("su");
				DataOutputStream os = new DataOutputStream(process.getOutputStream());
				DataInputStream osRes = new DataInputStream(process.getInputStream());
				for (String single : commands) {
					os.writeBytes(single + "\n");
					os.flush();
					Log.i(TAG, osRes.readLine());
				}
				os.writeBytes("exit\n");
				os.flush();
				process.waitFor();
								
			} catch (IOException e) {
				throw new RuntimeException(e);
			} catch (InterruptedException e) {
				throw new RuntimeException(e);
			}
        } 
    }
}




