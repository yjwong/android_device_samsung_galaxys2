package com.teamhacksung.volumehack.receiver;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;

import com.teamhacksung.volumehack.service.HeadsetService;

public class BootCompletedReceiver extends BroadcastReceiver {
	@Override
	public void onReceive(Context context, Intent intent) {
		context.startService(new Intent(context, HeadsetService.class));
	}
}
