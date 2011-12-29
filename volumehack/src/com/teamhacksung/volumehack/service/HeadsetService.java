package com.teamhacksung.volumehack.service;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.UUID;

import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.media.AudioManager;
import android.os.Binder;
import android.os.IBinder;
import android.util.Log;

import com.teamhacksung.volumehack.receiver.HeadsetPlugReceiver;

public class HeadsetService extends Service {

	private static final String TAG = "VolumeHack";
	HeadsetPlugReceiver hspr;

    /**
     * Class for clients to access.  Because we know this service always
     * runs in the same process as its clients, we don't need to deal with
     * IPC.
     */
    public class LocalBinder extends Binder {
        HeadsetService getService() {
            return HeadsetService.this;
        }
    }
    
    @Override
    public void onCreate() {
		Log.i(TAG, "Starting service.");
		
		hspr = new HeadsetPlugReceiver();
        IntentFilter intf = new IntentFilter(); 
        intf.addAction("android.intent.action.HEADSET_PLUG"); 
        registerReceiver(hspr, intf); 
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        return START_STICKY;
    }

    @Override
    public void onDestroy() {
		Log.i(TAG, "Stopping service.");
		unregisterReceiver(hspr); 
    }

    @Override
    public IBinder onBind(Intent intent) {
        return mBinder;
    }

	// This is the object that receives interactions from clients.  See
    // RemoteService for a more complete example.
    private final IBinder mBinder = new LocalBinder();

}
