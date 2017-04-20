package com.gpac.Osmo4;

import android.util.Log;

import com.qualcomm.vuforia.Vuforia;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;

import javax.xml.parsers.*;

/**
 * Created by traian on 4/25/16.
 */
public class vuforiaInit {
    // Vuforia initialization flags:
    private int mVuforiaFlags;
    public static boolean vuforiaInitalized;

    private static Osmo4 osmoInstance;

    public static native int initTracker();
    public static native void deInitTracker();

    public static void setOsmo4Instance (Osmo4 osmoInst) {
        osmoInstance = osmoInst;
    //    Log.i("[vuforiaInit]", "setOsmo4Instance");
    }

    public vuforiaInit () {
        mVuforiaFlags = 0;
        vuforiaInitalized = false;
        initializeVuforia();
        initTracker();
    }

    public void initializeVuforia () {
        mVuforiaFlags = Vuforia.GL_20;
        Vuforia.setInitParameters(osmoInstance, mVuforiaFlags, "AUyQSKv/////AAAAAToGKyLTS0d3lgI58o6de9kRzOFoJ11DFiBf+Dm8B6eWEp6Nj9w4r77spvsrKk12ROygNop9YjTc79FZmnqGHgjAWGi42Wd3mXQzMKTqJ//YS1vL1gG+lUjUDfWMNmSQgXigqLJ9yvxQ3wz3qqjIdWyRu05Xm6ZHrwhwByrNDV66DQywnkP11pSxbkWwR+Ft4hs2xsBER5InKmcM4CWP8S4PS7w6A4hy/mhupKlQE2NGRSoCiXgGtPwVQwoFjY+NtuyNyOVYs6BYIlCVOsjlHUW3QeKsAOrFEpM5c5MNYfIyKxqTP/tPNrQNuqxZr0ASJBD838skwua/8qYc6UcfnyEbtBBZJ/PLnpkxpS9Mf2Yl");
        int mProgressValue;
        do
        {
            mProgressValue = Vuforia.init();

        } while (mProgressValue >= 0 && mProgressValue < 100);
        vuforiaInitalized = true;
    }

    public String getReferenceNames(String XMLPath) {
        Log.i("[vuforiaInit]", "Using the XML file: " + XMLPath);
        String imageTargetNames = "";

        try {
            File file = new File(XMLPath);
            InputStream inputSource = new FileInputStream(file.getPath());
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(inputSource);
            Element element = doc.getDocumentElement();
            element.normalize();
            NodeList imageTargetList = doc.getElementsByTagName("ImageTarget");


            for (int i = 0; i < imageTargetList.getLength(); i++) {
                Node imageTarget = imageTargetList.item(i);
                Element eElement = (Element) imageTarget;
                imageTargetNames += eElement.getAttributes().getNamedItem("name").getNodeValue() + " ";
            }

        } catch (Exception e) {
            e.printStackTrace();
            Log.e("[vuforiaInit]", "The XML file " + XMLPath + " could not be found!");
        }
        Log.i("[vuforiaInit]", "Target names found in the XML file: " + imageTargetNames);
        return imageTargetNames;
    }
}
