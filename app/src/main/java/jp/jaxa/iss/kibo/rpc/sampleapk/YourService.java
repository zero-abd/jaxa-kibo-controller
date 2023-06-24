package jp.jaxa.iss.kibo.rpc.sampleapk;

import java.util.Arrays;
import java.util.ArrayList;

import android.graphics.Bitmap;
import android.util.Log;

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
                {0, 42, 20, 50, 60, 47, 29, 50, 1000},
                {0, 0, 48, 51, 44, 37, 26, 31, 56},
                {0, 46, 0, 44, 58, 42, 38, 45, 51},
                {0, 51, 47, 0, 41, 42, 44, 45, 25},
                {0, 45, 56, 41, 0, 30, 56, 53, 19},
                {0, 34, 45, 42, 29, 0, 37,  37, 25},
                {0, 26, 38, 43, 54, 56, 0, 22, 55},
                {0, 32, 46, 42, 53, 37, 24, 0, 43}
        };

        List<Long> timeRemaining;
        List<Integer> active_points;
        Route best_path;
        Route best_path_qr;
        Long max_time;
        int current_point = 0;
        boolean qr_ready = false;
        String final_message = "I_AM_HERE";


        while(true){
            active_points = api.getActiveTargets();
            timeRemaining = api.getTimeRemaining();
            if(active_points.contains(6) && !qr_ready) active_points.add(7);
            best_path = getBestRoute(current_point, active_points, timeRemaining.get(1));

            if(best_path.usable){
                timeRemaining = api.getTimeRemaining();
                max_time = (timeRemaining.get(1) / 1000) - 6;
                if(max_time - best_path.timeWithGoal <= 40){
                    active_points = best_path.listWithGoal;
                }else{
                    active_points = best_path.listWithoutGoal;
                }
            }else{
                break;
            }


            for(int i = 0; i < active_points.size(); i++){
                int target_point = active_points.get(i);
                timeRemaining = api.getTimeRemaining();
                max_time = (timeRemaining.get(1) / 1000) - 6;
                if(move_time[current_point][target_point] + move_time[target_point][8] >= max_time) break;


                if(target_point == 7 && !qr_ready){
                    moveFromTo(current_point, 7);
                    final_message = getQrMessage();
                    current_point = 7;
                    qr_ready = true;
                    continue;
                }

                moveFromTo(current_point, target_point);
                current_point = target_point;

//                takePic(current_point);
                shootLaser(current_point);
            }
        }

        timeRemaining = api.getTimeRemaining();
        max_time = (timeRemaining.get(1) / 1000) - 10;
        if(move_time[current_point][7] + move_time[7][8] <= max_time && !qr_ready){
            moveFromTo(current_point, 7);
            final_message = getQrMessage();
            timeRemaining = api.getTimeRemaining();
            Log.i(TAG,"getting qr at: " + timeRemaining.get(1));
            current_point = 7;
            qr_ready = true;
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
    private void moveToGoal(int idx, String message){
        api.notifyGoingToGoal();
        moveFromTo(idx, 8);  // 8 is the goal point;
        api.reportMissionCompletion(message);
    }

    private void moveFromTo(int a, int b){
        Log.i(TAG, "Moving from point " + a + " to " + b);
        // define all the points and quaternions
        Point start_point = new Point(9.815, -9.806, 4.293);
        Point undock_point = new Point(10.4, -9.806, 4.293); // 0
        Point goal_point = new Point(11.143, -6.7607, 4.9654); // 8
        Point point_1 = new Point(11.2036, -9.92284, 5.4748);
        Point point_2 = new Point(10.452, -9.1809, 4.48);
        Point point_3 = new Point(10.72, -7.767, 4.48);
        Point point_4 = new Point(10.51, -6.6085, 5.2104);
        Point point_5 = new Point(11.049, -7.9206, 5.3393);
        Point point_6 = new Point(11.355, -9.0319, 4.9348);
        Point point_7 = new Point(11.4316, -8.5096, 4.48);

        Quaternion start_quaternion = new Quaternion(1f, 0f, 0f, 0f);
        Quaternion goal_quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        Quaternion quaternion_1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        Quaternion quaternion_2 = new Quaternion(0.5f, 0.5f, -0.5f, 0.5f);
        Quaternion quaternion_3 = new Quaternion(0f, 0.707f, 0f, 0.707f);
        Quaternion quaternion_4 = new Quaternion(0f, 0f, -1f, 0f);
        Quaternion quaternion_5 = new Quaternion(-0.5f, -0.5f, -0.5f, 0.5f);
        Quaternion quaternion_6 = new Quaternion(0f, 0f, 0f, 1f);
        Quaternion quaternion_7 = new Quaternion(0f, 0.707f, 0f, 0.707f);

        if(a == 0){
            if(b == 1){
                Point zero_one_step_1 = new Point(10.6, -9.806, 5.1);
                moveToPoint(zero_one_step_1, quaternion_1);
                moveToPoint(point_1, quaternion_1);
            }else if(b == 2){
                moveToPoint(point_2, quaternion_2);
            }else if(b == 3){
                Point zero_three_step_1 = new Point(10.72, -8.2826, 4.7525);
                moveToPoint(zero_three_step_1, quaternion_3);
                moveToPoint(point_3, quaternion_3);
            }else if(b == 4){
                moveToPoint(point_4, quaternion_4);
            }else if(b == 5){
                Point zero_five_step_1 = new Point(11.114, -8.28, 5.3393);
                moveToPoint(zero_five_step_1, quaternion_5);
                moveToPoint(point_5, quaternion_5);
            }else if(b == 6){
                moveToPoint(point_6, quaternion_6);
            }else if(b == 7){
                Point zero_seven_step_1 = new Point(10.683, -9.7929, 4.48);
                moveToPoint(zero_seven_step_1, start_quaternion);
                moveToPoint(point_7, quaternion_7);
            }
        }else if(a == 1){
            if(b == 2){
                Point one_two_step_1 = new Point(10.612, -9.0709, 5.2988);
                moveToPoint(one_two_step_1, quaternion_2);
                moveToPoint(point_2, quaternion_2);
            }else if(b == 3){
                Point one_three_step_1 = new Point(10.71, -8.28, 4.95);
                moveToPoint(one_three_step_1, quaternion_3);
                moveToPoint(point_3, quaternion_3);
            }else if(b == 4){
                moveToPoint(point_4, quaternion_4);
            }else if(b == 5){
                moveToPoint(point_5, quaternion_5);
            }else if(b == 6){
                moveToPoint(point_6, quaternion_6);
            }else if(b == 7){
                moveToPoint(point_7, quaternion_7);
            }else if(b == 8){
                Point one_goal_step_1 = new Point(11.143, -7.7405, 5.34);
                moveToPoint(one_goal_step_1, goal_quaternion);
                moveToPoint(goal_point, goal_quaternion);
            }
        }else if(a == 2){
            if(b == 1){
                Point two_one_step_1 = new Point(10.612, -9.0709, 5.2988);
                moveToPoint(two_one_step_1, quaternion_1);
                moveToPoint(point_1, quaternion_1);
            }else if(b == 3){
                Point two_three_step_1 = new Point(10.71, -8.28, 4.95);
                moveToPoint(two_three_step_1, quaternion_3);
                moveToPoint(point_3, quaternion_3);
            }else if(b == 4){
                Point two_four_step_1 = new Point(10.71, -8.28, 4.95);
                moveToPoint(two_four_step_1, quaternion_4);
                moveToPoint(point_4, quaternion_4);
            }else if(b == 5){
                Point two_five_step_1 = new Point(11.114, -8.28, 5.3393);
                moveToPoint(two_five_step_1, quaternion_5);
                moveToPoint(point_5, quaternion_5);
            }else if(b == 6){
                Point two_six_step_1 = new Point(10.62, -8.9929, 4.9);
                moveToPoint(two_six_step_1, quaternion_2);
                moveToPoint(point_6, quaternion_6);
            }else if(b == 7){
                Point two_seven_step_1 = new Point(10.62, -8.5518, 5);
                moveToPoint(two_seven_step_1, quaternion_2);
                moveToPoint(point_7, quaternion_7);
            }else if(b == 8){
                Point two_goal_step_1 = new Point(10.71, -8.28, 4.93);
                moveToPoint(two_goal_step_1, quaternion_2);
                moveToPoint(goal_point, goal_quaternion);
            }
        }else if(a == 3){
            if(b == 1){
                Point three_one_step_1 = new Point(10.71, -8.28, 4.95);
                moveToPoint(three_one_step_1, quaternion_1);
                moveToPoint(point_1, quaternion_1);
            }else if(b == 2){
                Point three_two_step_1 = new Point(10.71, -8.28, 4.95);
                moveToPoint(three_two_step_1, quaternion_2);
                moveToPoint(point_2, quaternion_2);
            }else if(b == 4){
                Point three_four_step_1 = new Point(10.51, -7.1449, 5.1804);
                moveToPoint(three_four_step_1, quaternion_4);
                moveToPoint(point_4, quaternion_4);
            }else if(b == 5){
                Point three_five_step_1 = new Point(10.56, -7.9756, 5.3393);
                moveToPoint(three_five_step_1, quaternion_5);
                moveToPoint(point_5, quaternion_5);
            }else if(b == 6){
                Point three_six_step_1 = new Point(11.355, -8.2826, 4.91);
                moveToPoint(three_six_step_1, quaternion_6);
                moveToPoint(point_6, quaternion_6);
            }else if(b == 7){
                Point three_seven_step_1 = new Point(11.369, -8.2826, 4.91);
                moveToPoint(three_seven_step_1, quaternion_7);
                moveToPoint(point_7, quaternion_7);
            }else if(b == 8){
                moveToPoint(goal_point, goal_quaternion);
            }
        }else if(a == 4){
            if(b == 1){
                moveToPoint(point_1, quaternion_1);
            }else if(b == 2){
                Point four_two_step_1 = new Point(10.71, -8.28, 4.95);
                moveToPoint(four_two_step_1, quaternion_2);
                moveToPoint(point_2, quaternion_2);
            }else if(b == 3){
                Point four_three_step_1 = new Point(10.51, -7.1449, 5.1804);
                moveToPoint(four_three_step_1, quaternion_4);
                moveToPoint(point_3, quaternion_3);
            }else if(b == 5){
                moveToPoint(point_5, quaternion_5);
            }else if(b == 6){
                Point four_six_step_1 = new Point(10.71, -8.28, 4.95);
                moveToPoint(four_six_step_1, quaternion_6);
                moveToPoint(point_6, quaternion_6);
            }else if(b == 7){
                Point four_seven_step_1 = new Point(10.71, -8.28, 4.95);
                moveToPoint(four_seven_step_1, quaternion_7);
                moveToPoint(point_7, quaternion_7);
            }else if(b == 8){
                moveToPoint(goal_point, goal_quaternion);
            }
        }else if(a == 5){
            if(b == 1){
                moveToPoint(point_1, quaternion_1);
            }else if(b == 2){
                Point five_two_step_1 = new Point(11.114, -8.28, 5.3393);
                moveToPoint(five_two_step_1, quaternion_5);
                moveToPoint(point_2, quaternion_2);
            }else if(b == 3){
                Point five_three_step_1 = new Point(10.56, -7.9756, 5.3393);
                moveToPoint(five_three_step_1, quaternion_5);
                moveToPoint(point_3, quaternion_3);
            }else if(b == 4){
                moveToPoint(point_4, quaternion_4);
            }else if(b == 6){
                Point five_six_step_1 = new Point(11.114, -8.28, 5.3393);
                moveToPoint(five_six_step_1, quaternion_5);
                moveToPoint(point_6, quaternion_6);
            }else if(b == 7){
                Point five_seven_step_1 = new Point(11.114, -8.28, 5.3393);
                moveToPoint(five_seven_step_1, quaternion_5);
                moveToPoint(point_7, quaternion_7);
            }else if(b == 8){
                moveToPoint(goal_point, goal_quaternion);
            }
        }else if(a == 6){
            if(b == 1){
                moveToPoint(point_1, quaternion_1);
            }else if(b == 2){
                Point six_two_step_1 = new Point(10.62, -8.9929, 4.9);
                moveToPoint(six_two_step_1, quaternion_2);
                moveToPoint(point_2, quaternion_2);
            }else if(b == 3){
                Point six_three_step_1 = new Point(11.355, -8.2826, 4.91);
                moveToPoint(six_three_step_1, quaternion_6);
                moveToPoint(point_3, quaternion_3);
            }else if(b == 4){
                Point six_four_step_1 = new Point(10.71, -8.28, 4.95);
                moveToPoint(six_four_step_1, quaternion_4);
                moveToPoint(point_4, quaternion_4);
            }else if(b == 5){
                Point six_five_step_1 = new Point(11.114, -8.28, 5.3393);
                moveToPoint(six_five_step_1, quaternion_6);
                moveToPoint(point_5, quaternion_5);
            }else if(b == 7){
                moveToPoint(point_7, quaternion_7);
            }else if(b == 8){
                Point six_goal_step_1 = new Point(11.355, -8.2826, 4.91);
                moveToPoint(six_goal_step_1, quaternion_6);
                moveToPoint(goal_point, goal_quaternion);
            }
        }else if(a == 7){
            if(b == 1){
                moveToPoint(point_1, quaternion_1);
            }else if(b == 2){
                Point seven_two_step_1 = new Point(10.62, -8.5518, 5);
                moveToPoint(seven_two_step_1, quaternion_2);
                moveToPoint(point_2, quaternion_2);
            }else if(b == 3){
                Point seven_three_step_1 = new Point(11.369, -8.3826, 4.91);
                moveToPoint(seven_three_step_1, quaternion_3);
                moveToPoint(point_3, quaternion_3);
            }else if(b == 4){
                Point seven_four_step_1 = new Point(10.71, -8.28, 4.95);
                moveToPoint(seven_four_step_1, quaternion_4);
                moveToPoint(point_4, quaternion_4);
            }else if(b == 5){
                Point seven_five_step_1 = new Point(11.114, -8.28, 5.3393);
                moveToPoint(seven_five_step_1, quaternion_5);
                moveToPoint(point_5, quaternion_5);
            }else if(b == 6){
                moveToPoint(point_6, quaternion_6);
            }else if(b == 8){
                Point seven_goal_step_1 = new Point(11.355, -8.3826, 4.91);
                moveToPoint(seven_goal_step_1, quaternion_7);
                moveToPoint(goal_point, goal_quaternion);
            }
        }


        Log.i(TAG, "Successfully Moved from point " + a + " to " + b);
    }

    private void moveToPoint(Point p, Quaternion q){
        Result r = api.moveTo(p, q, true);

        final int cnt_max = 2;
        int cnt = 0;
        while(!r.hasSucceeded() && cnt < cnt_max){
            // retry
            r = api.moveTo(p, q, true);
            cnt++;
        }
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
        Log.i(TAG, "successfully completed task at point " + idx);
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
        Bitmap bitmap = api.getBitmapNavCam();
        api.saveBitmapImage(bitmap, "qr_image.png");

        int width = bitmap.getWidth();
        int height = bitmap.getHeight();

        int[] pixels = new int[width * height];

        bitmap.getPixels(pixels, 0, width, 0, 0, width, height);

        RGBLuminanceSource source = new RGBLuminanceSource(width, height, pixels);
        BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));

        try {
            com.google.zxing.Result result = new MultiFormatReader().decode(binaryBitmap);

            if (result == null){
                Log.i(TAG, "failed in step 1");
                return "ASTROBEE";
            } else {
                return result.getText();
            }
        }catch (NotFoundException e){
            Log.i(TAG, "failed in step 2");
            return "ASTROBEE";
        }
    }


    // getting the best possible path in a phase
    private int[] timeAndScore(int start_point, List<Integer> path){
        int[][] move_time = {
                {0, 42, 20, 50, 60, 47, 29, 50, 1000},
                {0, 0, 48, 51, 44, 37, 26, 31, 56},
                {0, 46, 0, 44, 58, 42, 38, 45, 51},
                {0, 51, 47, 0, 41, 42, 44, 45, 25},
                {0, 45, 56, 41, 0, 30, 56, 53, 19},
                {0, 34, 45, 42, 29, 0, 37,  0, 37},
                {0, 26, 38, 43, 54, 56, 0, 22, 55},
                {0, 32, 46, 42, 53, 37, 24, 0, 43}
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
        Log.i(TAG,"scores at timeAndScore:  " + final_score + "  " + final_time + "  " + move_time[current_point][8]);
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
        bestRoute.timeWithGoal = 1000;
        bestRoute.timeWithoutGoal = 1000;
        bestRoute.score = 0;
        bestRoute.usable = false;
        Long max_time = (remainingTime / 1000) - 6; // safety time

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
            Log.i(TAG,"time_score without goal: " + time_score[0] + "  " + time_score[1] + "  " + time_score[2]);
        }
        if(time_score[2] <= bestRoute.timeWithGoal){
            bestRoute.timeWithGoal = time_score[2];
            bestRoute.score = time_score[0];
            bestRoute.listWithGoal = list;
            Log.i(TAG,"time_score with goal: " + time_score[0] + "  " + time_score[1] + "  " + time_score[2]);
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
                    Log.i(TAG,"time_score without goal: " + time_score[0] + "  " + time_score[1] + "  " + time_score[2]);
                }
                if(time_score[2] <= bestRoute.timeWithGoal){
                    bestRoute.timeWithGoal = time_score[2];
                    bestRoute.score = time_score[0];
                    bestRoute.listWithGoal = list;
                    Log.i(TAG,"time_score with goal: " + time_score[0] + "  " + time_score[1] + "  " + time_score[2]);
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
