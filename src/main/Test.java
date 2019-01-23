package main;

import java.util.LinkedHashMap;

public class Test {

    public static LinkedHashMap<Integer, LinkedHashMap<Integer, Integer>> occurrences = new LinkedHashMap<>();
//    public static int getValue(int x, int y) {
//        if (!occurrences.containsKey(x)) {
//            System.out.println(x + " not found");
//            occurrences.put(x, new LinkedHashMap<>());
//            return 0;
//        }
//        LinkedHashMap<Integer, Integer> pts = occurrences.get(x);
//
//        Integer v = pts.get(y);
//        if (v==null) {
//            System.out.println("(" + x + "," + y + ") not found");
//            return 0;
//        }
//
//        return v;
//    }

    public static void countOcc(int x, int y) {
        if (!occurrences.containsKey(x)) {
            System.out.println("creating new for " + x);
            occurrences.put(x, new LinkedHashMap<>());
        }

        LinkedHashMap<Integer, Integer> vals = occurrences.get(x);
        vals.merge(y, 1, (a, b) -> a + b);
    }

    public static void main(String[] args) {
//        System.out.println("0,0 = " + getValue(0,0));
//        countOcc(0,0);
//        System.out.println("0,0 = " + getValue(0,0));
//        System.out.println("0,0 = " + getValue(0,0));
//        System.out.println("0,0 = " + getValue(0,0));
//
//        countOcc(0,0);
//        countOcc(0,0);
//        countOcc(0,0);
//        countOcc(0,0);
//        System.out.println("0,0 = " + getValue(0,0));
//        System.out.println("0,0 = " + getValue(0,0));
    }
}
