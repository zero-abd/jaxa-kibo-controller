package jp.jaxa.iss.kibo.rpc.sampleapk;

import java.util.Arrays;
import java.util.ArrayList;

import android.graphics.Bitmap;
import android.util.Log;

import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.android.gs.MessageType;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.Mat;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;


public class YourService extends KiboRpcService {

    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1(){
        Log.i(TAG, "Team paragon starts their mission");
        // Team Paragon starts their mission
        api.startMission();


        int[][] move_time = {
                {0, 29, 22, 46, 43, 35, 30, 46, 1000},
                {0, 0, 27, 35, 42, 35, 24, 29, 36},
                {0, 28, 0, 30, 39, 31, 26, 33, 34},
                {0, 35, 30, 0, 27, 38, 30, 24, 25},
                {0, 42, 39, 28, 0, 29, 38, 35, 17},
                {0, 35, 31, 38, 28, 0, 28, 32, 27},
                {0, 24, 26, 30, 38, 27, 0, 19, 48},
                {0, 27, 32, 23, 33, 31, 17, 0, 30}
        };

        List<Long> timeRemaining;
        List<Integer> active_points;
        List<Integer> temp_active_points;
        Route best_path;
        Long max_time;
        int current_point = 0;
        int prev_point = 0;
        int sp_case = 0;
        boolean qr_ready = false;
        String final_message = "I_AM_HERE";

        try {
            while (true) {
                active_points = api.getActiveTargets();
                timeRemaining = api.getTimeRemaining();
                if(active_points.contains(6) && !qr_ready) active_points.add(7);
                best_path = getBestRoute(current_point, active_points, timeRemaining.get(1));

                if (best_path.usable) {
                    timeRemaining = api.getTimeRemaining();
                    max_time = (timeRemaining.get(1) / 1000) - 4;
                    if (max_time - best_path.timeWithGoal <= 40) {
                        active_points = best_path.listWithGoal;
                    } else {
                        active_points = best_path.listWithoutGoal;
                    }
                } else {
                    break;
                }


                for (int i = 0; i < active_points.size(); i++) {
                    int target_point = active_points.get(i);
                    timeRemaining = api.getTimeRemaining();
                    max_time = (timeRemaining.get(1) / 1000) - 3;
                    if (move_time[current_point][target_point] + move_time[target_point][8] >= max_time)
                        break;


                    if (target_point == 7 && !qr_ready) {
                        if (moveFromTo(current_point, 7, 0)) {
                            final_message = getQrMessage();
                            current_point = 7;
                            qr_ready = true;
                            robotKinematics();
                        }
                        continue;
                    }

                    if (prev_point == 0 && current_point == 1 && target_point == 2) sp_case = 1;
                    else sp_case = 0;
                    if (moveFromTo(current_point, target_point, sp_case)) {
                        robotKinematics();
                        prev_point = current_point;
                        current_point = target_point;
                        //  takePic(current_point);
                        int cnt_max = 2;
                        int cnt = 0;
                        while (cnt < cnt_max) {
                            shootLaser(current_point);
                            temp_active_points = api.getActiveTargets();
                            if (!temp_active_points.contains(current_point)) {
                                break;
                            }
                            cnt++;
                        }
                        temp_active_points = api.getActiveTargets();
                        if (temp_active_points.contains(current_point)) {
                            if (current_point == 1) {
                                Log.i(TAG, "Initiating sp 1");
                                Point case_sp1 = new Point(11.2036, -9.92284, 5.4748);
                                Point point_sp1 = new Point(11.2036, -9.72284, 5.4748);
                                Quaternion quaternion_sp1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
                                moveToPoint(case_sp1, quaternion_sp1);
                                shootLaser(current_point);
                                moveToPoint(point_sp1, quaternion_sp1);
                            } else if (current_point == 2) {
                                Log.i(TAG, "Initiating sp 2");
                                Point case_sp2 = new Point(10.452, -9.1809, 4.48);
                                Point point_sp2 = new Point(10.452, -9.1809, 4.67);
                                Quaternion quaternion_sp2 = new Quaternion(0.5f, 0.5f, -0.5f, 0.5f);
                                moveToPoint(case_sp2, quaternion_sp2);
                                shootLaser(current_point);
                                moveToPoint(point_sp2, quaternion_sp2);
                            } else if (current_point == 3) {
                                Log.i(TAG, "Initiating sp 3");
                                Point case_sp3 = new Point(10.72, -7.767, 4.48);
                                Point point_sp3 = new Point(10.72, -7.767, 4.73);
                                Quaternion quaternion_sp3 = new Quaternion(0f, 0.707f, 0f, 0.707f);
                                moveToPoint(case_sp3, quaternion_sp3);
                                shootLaser(current_point);
                                moveToPoint(point_sp3, quaternion_sp3);
                            }
                        }
                    }
                }
            }
        }catch (Exception xxx){
            Log.i(TAG, "Main loop failed");
        }
        try {
            timeRemaining = api.getTimeRemaining();
            max_time = (timeRemaining.get(1) / 1000) - 10;
            if (move_time[current_point][7] + move_time[7][8] <= max_time && !qr_ready) {
                moveFromTo(current_point, 7, 0);
                final_message = getQrMessage();
                timeRemaining = api.getTimeRemaining();
                Log.i(TAG, "getting qr at: " + timeRemaining.get(1));
                current_point = 7;
                qr_ready = true;
            }
        }catch (Exception xxxx){
            Log.i(TAG, "Main qr failed");
        }

        Log.i(TAG,"mission completion text: " + final_message);
        moveToGoal(current_point, final_message);
    }

    @Override
    protected void runPlan2(){
    }

    @Override
    protected void runPlan3(){
    }


    // movement and koz avoidance
    private void robotKinematics(){
        try {
            Kinematics kin = api.getRobotKinematics();
            Log.i(TAG, "Confidence now: " + kin.getConfidence());
        }catch (Exception e){
            Log.i(TAG, "Can't get kinematics");
        }
    }

    private void moveToGoal(int idx, String message){
        api.notifyGoingToGoal();
        moveFromTo(idx, 8, 0);  // 8 is the goal point;
        api.reportMissionCompletion(message);
    }

    private boolean moveFromTo(int a, int b, int sp_case){
        if(a == b) return true;

        Log.i(TAG, "Moving from point " + a + " to " + b);
        // define all the points and quaternions
        Point start_point = new Point(9.815, -9.806, 4.293);
        Point undock_point = new Point(10.4, -9.806, 4.293); // 0
        Point goal_point = new Point(11.143, -6.7607, 4.9654); // 8
        Point point_1 = new Point(11.2036, -9.72284, 5.4748);
        Point point_2 = new Point(10.452, -9.1809, 4.67);
        Point point_3 = new Point(10.72, -7.767, 4.73);
        Point point_4 = new Point(10.72, -6.6085, 5.2104);
        Point point_5 = new Point(11.049, -7.9206, 5.3393);
        Point point_6 = new Point(11.4316, -9.0319, 4.9348);
        Point point_7 = new Point(11.4316, -8.5096, 4.73);

        Quaternion start_quaternion = new Quaternion(1f, 0f, 0f, 0f);
        Quaternion goal_quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        Quaternion quaternion_1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        Quaternion quaternion_2 = new Quaternion(0.5f, 0.5f, -0.5f, 0.5f);
        Quaternion quaternion_3 = new Quaternion(0f, 0.707f, 0f, 0.707f);
        Quaternion quaternion_4 = new Quaternion(0f, 0f, -1f, 0f);
        Quaternion quaternion_5 = new Quaternion(-0.5f, -0.5f, -0.5f, 0.5f);
        Quaternion quaternion_6 = new Quaternion(0f, 0f, 0f, 1f);
        Quaternion quaternion_7 = new Quaternion(0f, 0.707f, 0f, 0.707f);

        boolean success = true;
        if(a == 0){
            if(b == 1){
                Point zero_one_step_1 = new Point(11.2036, -10.1, 5.4748);
                success &= moveToPoint(zero_one_step_1, quaternion_1);
//                success &= moveToPoint(point_1, quaternion_1);
            }else if(b == 2){
                success &= moveToPoint(point_2, quaternion_2);
            }else if(b == 3){
                Point zero_three_step_1 = new Point(10.72, -8.2826, 4.7525);
                success &= moveToPoint(zero_three_step_1, quaternion_3);
                success &= moveToPoint(point_3, quaternion_3);
            }else if(b == 4){
                success &= moveToPoint(point_4, quaternion_4);
            }else if(b == 5){
                success &= moveToPoint(point_5, quaternion_5);
            }else if(b == 6){
                success &= moveToPoint(point_6, quaternion_6);
            }else if(b == 7){
                Point zero_seven_step_1 = new Point(10.683, -9.7929, 4.48);
                success &= moveToPoint(zero_seven_step_1, start_quaternion);
                success &= moveToPoint(point_7, quaternion_7);
            }
        }else if(a == 1){
            if(b == 2){
                if (sp_case == 1){
                    Point one_two_step_1 = new Point(10.612, -9.0709, 5.2988);
                    success &= moveToPoint(one_two_step_1, quaternion_2);
                }
                success &= moveToPoint(point_2, quaternion_2);
            }else if(b == 3){
                success &= moveToPoint(point_3, quaternion_3);
            }else if(b == 4){
                success &= moveToPoint(point_4, quaternion_4);
            }else if(b == 5){
                success &= moveToPoint(point_5, quaternion_5);
            }else if(b == 6){
                success &= moveToPoint(point_6, quaternion_6);
            }else if(b == 7){
                success &= moveToPoint(point_7, quaternion_7);
            }else if(b == 8){
                Point one_goal_step_1 = new Point(11.143, -6.7607, 5.15);
                success &= moveToPoint(one_goal_step_1, goal_quaternion);
//                success &= moveToPoint(goal_point, goal_quaternion);
            }
        }else if(a == 2){
            if(b == 1){
                success &= moveToPoint(point_1, quaternion_1);
            }else if(b == 3){
                success &= moveToPoint(point_3, quaternion_3);
            }else if(b == 4){
                success &= moveToPoint(point_4, quaternion_4);
            }else if(b == 5){
                success &= moveToPoint(point_5, quaternion_5);
            }else if(b == 6){
                success &= moveToPoint(point_6, quaternion_6);
            }else if(b == 7){
                Point two_five_step_1 = new Point(10.452, -9.1809, 4.77);
                success &= moveToPoint(two_five_step_1, quaternion_2);
                success &= moveToPoint(point_7, quaternion_7);
            }else if(b == 8){
                success &= moveToPoint(goal_point, goal_quaternion);
            }
        }else if(a == 3){
            if(b == 1){
                success &= moveToPoint(point_1, quaternion_1);
            }else if(b == 2){
                success &= moveToPoint(point_2, quaternion_2);
            }else if(b == 4){
                success &= moveToPoint(point_4, quaternion_4);
            }else if(b == 5){
                Point three_five_step_1 = new Point(10.68, -7.9206, 5.3393);
                success &= moveToPoint(three_five_step_1, quaternion_5);
                success &= moveToPoint(point_5, quaternion_5);
            }else if(b == 6){
                success &= moveToPoint(point_6, quaternion_6);
            }else if(b == 7){
                success &= moveToPoint(point_7, quaternion_7);
            }else if(b == 8){
                success &= moveToPoint(goal_point, goal_quaternion);
            }
        }else if(a == 4){
            if(b == 1){
                success &= moveToPoint(point_1, quaternion_1);
            }else if(b == 2){
                success &= moveToPoint(point_2, quaternion_2);
            }else if(b == 3){
                success &= moveToPoint(point_3, quaternion_3);
            }else if(b == 5){
                success &= moveToPoint(point_5, quaternion_5);
            }else if(b == 6){
                success &= moveToPoint(point_6, quaternion_6);
            }else if(b == 7){
                success &= moveToPoint(point_7, quaternion_7);
            }else if(b == 8){
                success &= moveToPoint(goal_point, goal_quaternion);
            }
        }else if(a == 5){
            if(b == 1){
                success &= moveToPoint(point_1, quaternion_1);
            }else if(b == 2){
                success &= moveToPoint(point_2, quaternion_2);
            }else if(b == 3){
                Point five_three_step_1 = new Point(10.68, -7.9206, 5.3393);
                success &= moveToPoint(five_three_step_1, quaternion_5);
                success &= moveToPoint(point_3, quaternion_3);
            }else if(b == 4){
                success &= moveToPoint(point_4, quaternion_4);
            }else if(b == 6){
                success &= moveToPoint(point_6, quaternion_6);
            }else if(b == 7){
                Point five_seven_step_1 = new Point(11.049, -8.1035, 5.3393);
                success &= moveToPoint(five_seven_step_1, quaternion_5);
                success &= moveToPoint(point_7, quaternion_7);
            }else if(b == 8){
                success &= moveToPoint(goal_point, goal_quaternion);
            }
        }else if(a == 6){
            if(b == 1){
                success &= moveToPoint(point_1, quaternion_1);
            }else if(b == 2){
                success &= moveToPoint(point_2, quaternion_2);
            }else if(b == 3){
                success &= moveToPoint(point_3, quaternion_3);
            }else if(b == 4){
                success &= moveToPoint(point_4, quaternion_4);
            }else if(b == 5){
                success &= moveToPoint(point_5, quaternion_5);
            }else if(b == 7){
                success &= moveToPoint(point_7, quaternion_7);
            }else if(b == 8){
                success &= moveToPoint(goal_point, goal_quaternion);
            }
        }else if(a == 7){
            if(b == 1){
                success &= moveToPoint(point_1, quaternion_1);
            }else if(b == 2){
                Point seven_two_step_1 = new Point(10.452, -9.1809, 4.77);
                success &= moveToPoint(seven_two_step_1, quaternion_2);
                success &= moveToPoint(point_2, quaternion_2);
            }else if(b == 3){
                success &= moveToPoint(point_3, quaternion_3);
            }else if(b == 4){
                success &= moveToPoint(point_4, quaternion_4);
            }else if(b == 5){
                Point seven_five_step_1 = new Point(11.049, -8.1035, 5.3393);
                success &= moveToPoint(seven_five_step_1, quaternion_5);
                success &= moveToPoint(point_5, quaternion_5);
            }else if(b == 6){
                success &= moveToPoint(point_6, quaternion_6);
            }else if(b == 8){
                success &= moveToPoint(goal_point, goal_quaternion);
            }
        }

        if(success) {
            Log.i(TAG, "Successfully moved from point " + a + " to " + b);
            return true;
        }else{
            Log.i(TAG, "Failed to move from point " + a + " to " + b);
            return false;
        }
    }

    private boolean moveToPoint(Point p, Quaternion q){
        int cnt_max = 4;
        int cnt = 0;
        while(cnt < cnt_max){
            // retry
            Result r = api.moveTo(p, q, true);
            if(r.hasSucceeded()){
                return true;
            }
            cnt++;
        }
        return false;
    }


    // taking pictures and shooting laser
    private void takePic(int idx){
        Log.i(TAG, "Trying to get an image");
        Mat image_point_1 = api.getMatNavCam();
        api.saveMatImage(image_point_1, "image-point-" + idx + ".png");
        Log.i(TAG, "Image saved successfully at point: " + idx);
    }

    private void shootLaser(int idx){
        Log.i(TAG, "Taking snapshot of point " + idx);
        api.laserControl(true);
        api.takeTargetSnapshot(idx);
        Log.i(TAG, "Completed task at point " + idx);
    }


    // getting the qr in the best way possible
    private String getQrMessage(){
        Dictionary convert_msg = new Hashtable();
        convert_msg.put("JEM", "STAY_AT_JEM");
        convert_msg.put("COLUMBUS", "GO_TO_COLUMBUS");
        convert_msg.put("RACK1", "CHECK_RACK_1");
        convert_msg.put("ASTROBEE", "I_AM_HERE");
        convert_msg.put("INTBALL", "LOOKING_FORWARD_TO_SEE_YOU");
        convert_msg.put("BLANK", "NO_PROBLEM");

        try{
            Thread.sleep(1000);
        }catch (Exception e){
            Log.i(TAG, "failed because of: " + e);
        }

        String given_text_msg = getZbarQr();

        return (String)convert_msg.get(given_text_msg);
    }

    private String getZbarQr(){
        try {
            Bitmap bitmap = api.getBitmapNavCam();
//            api.saveBitmapImage(bitmap, "qr_image.png");

            int width = bitmap.getWidth();
            int height = bitmap.getHeight();

            int[] pixels = new int[width * height];

            bitmap.getPixels(pixels, 0, width, 0, 0, width, height);

            RGBLuminanceSource source = new RGBLuminanceSource(width, height, pixels);
            BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));

            try {
                com.google.zxing.Result result = new MultiFormatReader().decode(binaryBitmap);

                if (result == null) {
                    Log.i(TAG, "qr failed in step 1");
                    return "ASTROBEE";
                } else {
                    return result.getText();
                }
            } catch (NotFoundException e) {
                Log.i(TAG, "qr failed in step 2");
                return "ASTROBEE";
            }
        }catch (Exception x){
            try{
                try{
                    api.flashlightControlFront(0.5f);
                }catch (Exception e){
                    Log.i(TAG, "flashlight failed to turn on");
                }
                Bitmap bitmap = api.getBitmapNavCam();
                try{
                    api.flashlightControlFront(0f);
                }catch (Exception e){
                    Log.i(TAG, "flashlight failed to turn off");
                }
//                api.saveBitmapImage(bitmap, "qr_image.png");

                int width = bitmap.getWidth();
                int height = bitmap.getHeight();

                int[] pixels = new int[width * height];

                bitmap.getPixels(pixels, 0, width, 0, 0, width, height);

                RGBLuminanceSource source = new RGBLuminanceSource(width, height, pixels);
                BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));

                try {
                    com.google.zxing.Result result = new MultiFormatReader().decode(binaryBitmap);

                    if (result == null){
                        Log.i(TAG, "qr failed in step 1");
                        return "ASTROBEE";
                    } else {
                        return result.getText();
                    }
                }catch (NotFoundException e){
                    Log.i(TAG, "qr failed in step 2");
                    return "ASTROBEE";
                }
            }catch (Exception y){
                Log.i(TAG, "failed in step all");
                return  "ASTROBEE";
            }
        }
    }


    // getting the best possible path in a phase
    private int[] timeAndScore(int start_point, List<Integer> path){
        int[][] move_time = {
                {0, 29, 22, 46, 43, 35, 30, 46, 1000},
                {0, 0, 27, 35, 42, 35, 24, 29, 36},
                {0, 28, 0, 30, 39, 31, 26, 33, 34},
                {0, 35, 30, 0, 27, 38, 30, 24, 25},
                {0, 42, 39, 28, 0, 29, 38, 35, 17},
                {0, 35, 31, 38, 28, 0, 28, 32, 27},
                {0, 24, 26, 30, 38, 27, 0, 19, 48},
                {0, 27, 32, 23, 33, 31, 17, 0, 30}
        };

        int[] score_point = {0, 30, 20, 40, 20, 30, 30, 20, 0};

        int final_time = 0;
        int final_score = 0;
        int current_point = start_point;
        for(int i = 0; i < path.size(); ++i){
            final_score += score_point[path.get(i)];
            final_time += move_time[current_point][path.get(i)];
            current_point = path.get(i);
        }
        int[] sol = {final_score, final_time, final_time+move_time[current_point][8]};
//        Log.i(TAG,"scores at timeAndScore:  " + final_score + "  " + final_time + "  " + move_time[current_point][8]);
        return sol;
    }

    private class Route{
        public boolean usable;
        public int timeWithGoal;
        public int timeWithoutGoal;
        public int score;
        public List<Integer> listWithGoal;
        public List<Integer> listWithoutGoal;
    }

    private Route getBestRoute(int start_point, List<Integer> set, Long remainingTime){
        Route bestRoute = new Route();
        bestRoute.timeWithGoal = 10000;
        bestRoute.timeWithoutGoal = 10000;
        bestRoute.score = 0;
        bestRoute.usable = false;
        Long max_time = (remainingTime / 1000) - 4; // safety time

        List<List<Integer>> powerSet = new ArrayList<>();

        for (int i = 0; i < (1 << set.size()); i++) {
            List<Integer> subset = new ArrayList<>();
            for (int j = 0; j < set.size(); j++) {
                if ((i & (1 << j)) != 0) {
                    subset.add(set.get(j));
                }
            }
            if(subset.size()>0) powerSet.add(subset);
        }

        for (List<Integer> subset : powerSet) {
            Route tempRoute = permuteAllPath(start_point, subset);
            if(tempRoute.timeWithGoal <= max_time && tempRoute.timeWithoutGoal < 115){
                if(tempRoute.score > bestRoute.score){
                    bestRoute = tempRoute;
                    bestRoute.usable = true;
                }else if(tempRoute.score == bestRoute.score){
                    if(tempRoute.timeWithoutGoal <= bestRoute.timeWithoutGoal) {
                        bestRoute = tempRoute;
                        bestRoute.usable = true;
                    }
                }
            }
        }
        return bestRoute;
    }

    private Route permuteAllPath(int start_point, List<Integer> list) {
        Route bestRoute = new Route();
        bestRoute.timeWithGoal = 1000;
        bestRoute.timeWithoutGoal = 1000;
        int n = list.size();
        int[] indexes = new int[n];
        Arrays.fill(indexes, 0);

        int[] time_score = timeAndScore(start_point, list);

        if(time_score[1] <= bestRoute.timeWithoutGoal){
            bestRoute.timeWithoutGoal = time_score[1];
            bestRoute.score = time_score[0];
            bestRoute.listWithoutGoal = list;
//            Log.i(TAG,"time_score without goal: " + time_score[0] + "  " + time_score[1] + "  " + time_score[2]);
        }
        if(time_score[2] <= bestRoute.timeWithGoal){
            bestRoute.timeWithGoal = time_score[2];
            bestRoute.score = time_score[0];
            bestRoute.listWithGoal = list;
//            Log.i(TAG,"time_score with goal: " + time_score[0] + "  " + time_score[1] + "  " + time_score[2]);
        }
        int i = 0;
        while (i < n) {
            if (indexes[i] < i) {
                swap(list, i % 2 == 0 ? 0 : indexes[i], i);

                time_score = timeAndScore(start_point, list);
                if(time_score[1] <= bestRoute.timeWithoutGoal){
                    bestRoute.timeWithoutGoal = time_score[1];
                    bestRoute.score = time_score[0];
                    bestRoute.listWithoutGoal = list;
//                    Log.i(TAG,"time_score without goal: " + time_score[0] + "  " + time_score[1] + "  " + time_score[2]);
                }
                if(time_score[2] <= bestRoute.timeWithGoal){
                    bestRoute.timeWithGoal = time_score[2];
                    bestRoute.score = time_score[0];
                    bestRoute.listWithGoal = list;
//                    Log.i(TAG,"time_score with goal: " + time_score[0] + "  " + time_score[1] + "  " + time_score[2]);
                }

                indexes[i]++;
                i = 0;
            } else {
                indexes[i] = 0;
                i++;
            }
        }
        return bestRoute;
    }

    private void swap(List<Integer> list, int i, int j) {
        int temp = list.get(i);
        list.set(i, list.get(j));
        list.set(j, temp);
    }

}
