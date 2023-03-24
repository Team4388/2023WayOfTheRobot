// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.Swerve;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.UtilityStructs.TimedOutput;

public class JoystickPlayback extends CommandBase {
  private final SwerveDrive            swerve;
  private       String                 filename;
  private       int                    mult       = 1;
  private       Scanner                input;
  private final ArrayList<TimedOutput> outputs      = new ArrayList<>();
  private       int                    counter      = 0;
  private       long                   startTime    = 0;
  private       long                   playbackTime = 0;
  private       int                    lastIndex;
  private       boolean                m_finished   = false; // ! find a better way


  private String path = 
 "0.000000,0.000000,0.000000,0.000000,0\n"
+"-0.022108,-0.999756,0.000000,0.000000,16\n"
+"-0.022043,-0.999757,0.000000,0.000000,32\n"
+"-0.021977,-0.999758,0.000000,0.000000,48\n"
+"-0.021911,-0.999760,0.000000,0.000000,64\n"
+"-0.021845,-0.999761,0.000000,0.000000,80\n"
+"-0.021778,-0.999763,0.000000,0.000000,96\n"
+"-0.021711,-0.999764,0.000000,0.000000,112\n"
+"-0.021644,-0.999766,0.000000,0.000000,128\n"
+"-0.021577,-0.999767,0.000000,0.000000,144\n"
+"-0.021509,-0.999769,0.000000,0.000000,160\n"
+"-0.021441,-0.999770,0.000000,0.000000,176\n"
+"-0.021373,-0.999772,0.000000,0.000000,192\n"
+"-0.021304,-0.999773,0.000000,0.000000,208\n"
+"-0.021235,-0.999775,0.000000,0.000000,224\n"
+"-0.021166,-0.999776,0.000000,0.000000,239\n"
+"-0.021097,-0.999777,0.000000,0.000000,254\n"
+"-0.021027,-0.999779,0.000000,0.000000,269\n"
+"-0.020957,-0.999780,0.000000,0.000000,284\n"
+"-0.020886,-0.999782,0.000000,0.000000,299\n"
+"-0.020816,-0.999783,0.000000,0.000000,314\n"
+"-0.020745,-0.999785,0.000000,0.000000,329\n"
+"-0.020674,-0.999786,0.000000,0.000000,344\n"
+"-0.020602,-0.999788,0.000000,0.000000,359\n"
+"-0.020530,-0.999789,0.000000,0.000000,374\n"
+"-0.020458,-0.999791,0.000000,0.000000,389\n"
+"-0.020386,-0.999792,0.000000,0.000000,404\n"
+"-0.020313,-0.999794,0.000000,0.000000,419\n"
+"-0.020240,-0.999795,0.000000,0.000000,434\n"
+"-0.020166,-0.999797,0.000000,0.000000,449\n"
+"-0.020093,-0.999798,0.000000,0.000000,464\n"
+"-0.020018,-0.999800,0.000000,0.000000,479\n"
+"-0.019944,-0.999801,0.000000,0.000000,494\n"
+"-0.019869,-0.999803,0.000000,0.000000,509\n"
+"-0.019794,-0.999804,0.000000,0.000000,524\n"
+"-0.019719,-0.999806,0.000000,0.000000,539\n"
+"-0.019643,-0.999807,0.000000,0.000000,554\n"
+"-0.019567,-0.999809,0.000000,0.000000,569\n"
+"-0.019491,-0.999810,0.000000,0.000000,584\n"
+"-0.019414,-0.999812,0.000000,0.000000,599\n"
+"-0.019337,-0.999813,0.000000,0.000000,614\n"
+"-0.019259,-0.999815,0.000000,0.000000,629\n"
+"-0.019182,-0.999816,0.000000,0.000000,644\n"
+"-0.019104,-0.999818,0.000000,0.000000,659\n"
+"-0.019025,-0.999819,0.000000,0.000000,673\n"
+"-0.018946,-0.999821,0.000000,0.000000,687\n"
+"-0.018867,-0.999822,0.000000,0.000000,701\n"
+"-0.018787,-0.999823,0.000000,0.000000,715\n"
+"-0.018708,-0.999825,0.000000,0.000000,729\n"
+"-0.018627,-0.999826,0.000000,0.000000,743\n"
+"-0.018547,-0.999828,0.000000,0.000000,757\n"
+"-0.018466,-0.999829,0.000000,0.000000,771\n"
+"-0.018384,-0.999831,0.000000,0.000000,785\n"
+"-0.018302,-0.999832,0.000000,0.000000,799\n"
+"-0.018220,-0.999834,0.000000,0.000000,813\n"
+"-0.018138,-0.999835,0.000000,0.000000,827\n"
+"-0.018055,-0.999837,0.000000,0.000000,841\n"
+"-0.017972,-0.999838,0.000000,0.000000,855\n"
+"-0.017888,-0.999840,0.000000,0.000000,869\n"
+"-0.017804,-0.999841,0.000000,0.000000,883\n"
+"-0.017719,-0.999843,0.000000,0.000000,897\n"
+"-0.017635,-0.999844,0.000000,0.000000,911\n"
+"-0.017549,-0.999846,0.000000,0.000000,925\n"
+"-0.017464,-0.999847,0.000000,0.000000,939\n"
+"-0.017378,-0.999849,0.000000,0.000000,953\n"
+"-0.017291,-0.999850,0.000000,0.000000,967\n"
+"-0.017204,-0.999852,0.000000,0.000000,981\n"
+"-0.017117,-0.999853,0.000000,0.000000,995\n"
+"-0.017029,-0.999855,0.000000,0.000000,1009\n"
+"-0.016941,-0.999856,0.000000,0.000000,1023\n"
+"-0.016853,-0.999858,0.000000,0.000000,1037\n"
+"-0.016764,-0.999859,0.000000,0.000000,1051\n"
+"-0.016674,-0.999861,0.000000,0.000000,1065\n"
+"-0.016584,-0.999862,0.000000,0.000000,1079\n"
+"-0.016494,-0.999864,0.000000,0.000000,1092\n"
+"-0.016403,-0.999865,0.000000,0.000000,1105\n"
+"-0.016312,-0.999867,0.000000,0.000000,1118\n"
+"-0.016221,-0.999868,0.000000,0.000000,1131\n"
+"-0.016129,-0.999870,0.000000,0.000000,1144\n"
+"-0.016036,-0.999871,0.000000,0.000000,1157\n"
+"-0.015943,-0.999873,0.000000,0.000000,1170\n"
+"-0.015850,-0.999874,0.000000,0.000000,1183\n"
+"-0.015756,-0.999876,0.000000,0.000000,1196\n"
+"-0.015661,-0.999877,0.000000,0.000000,1209\n"
+"-0.015567,-0.999879,0.000000,0.000000,1222\n"
+"-0.015471,-0.999880,0.000000,0.000000,1235\n"
+"-0.015376,-0.999882,0.000000,0.000000,1248\n"
+"-0.015279,-0.999883,0.000000,0.000000,1261\n"
+"-0.015183,-0.999885,0.000000,0.000000,1274\n"
+"-0.015086,-0.999886,0.000000,0.000000,1287\n"
+"-0.014988,-0.999888,0.000000,0.000000,1300\n"
+"-0.014890,-0.999889,0.000000,0.000000,1313\n"
+"-0.014791,-0.999891,0.000000,0.000000,1326\n"
+"-0.014692,-0.999892,0.000000,0.000000,1339\n"
+"-0.014592,-0.999894,0.000000,0.000000,1352\n"
+"-0.014492,-0.999895,0.000000,0.000000,1365\n"
+"-0.014391,-0.999896,0.000000,0.000000,1378\n"
+"-0.014290,-0.999898,0.000000,0.000000,1391\n"
+"-0.014188,-0.999899,0.000000,0.000000,1404\n"
+"-0.014086,-0.999901,0.000000,0.000000,1417\n"
+"-0.013983,-0.999902,0.000000,0.000000,1430\n"
+"-0.013880,-0.999904,0.000000,0.000000,1443\n"
+"-0.013776,-0.999905,0.000000,0.000000,1456\n"
+"-0.013672,-0.999907,0.000000,0.000000,1468\n"
+"-0.013567,-0.999908,0.000000,0.000000,1480\n"
+"-0.013461,-0.999909,0.000000,0.000000,1492\n"
+"-0.013355,-0.999911,0.000000,0.000000,1504\n"
+"-0.013249,-0.999912,0.000000,0.000000,1516\n"
+"-0.013142,-0.999914,0.000000,0.000000,1528\n"
+"-0.013034,-0.999915,0.000000,0.000000,1540\n"
+"-0.012926,-0.999916,0.000000,0.000000,1552\n"
+"-0.012817,-0.999918,0.000000,0.000000,1564\n"
+"-0.012707,-0.999919,0.000000,0.000000,1576\n"
+"-0.012597,-0.999921,0.000000,0.000000,1588\n"
+"-0.012487,-0.999922,0.000000,0.000000,1600\n"
+"-0.012375,-0.999923,0.000000,0.000000,1612\n"
+"-0.012264,-0.999925,0.000000,0.000000,1624\n"
+"-0.012151,-0.999926,0.000000,0.000000,1636\n"
+"-0.012019,-0.999928,0.000000,0.000000,1652\n"
+"-0.011867,-0.999930,0.000000,0.000000,1668\n"
+"-0.011715,-0.999931,0.000000,0.000000,1684\n"
+"-0.011560,-0.999933,0.000000,0.000000,1700\n"
+"-0.011405,-0.999935,0.000000,0.000000,1716\n"
+"-0.011249,-0.999937,0.000000,0.000000,1732\n"
+"-0.011092,-0.999938,0.000000,0.000000,1748\n"
+"-0.010933,-0.999940,0.000000,0.000000,1764\n"
+"-0.010773,-0.999942,0.000000,0.000000,1780\n"
+"-0.010612,-0.999944,0.000000,0.000000,1796\n"
+"-0.010450,-0.999945,0.000000,0.000000,1812\n"
+"-0.010287,-0.999947,0.000000,0.000000,1827\n"
+"-0.010122,-0.999949,0.000000,0.000000,1842\n"
+"-0.009956,-0.999950,0.000000,0.000000,1857\n"
+"-0.009789,-0.999952,0.000000,0.000000,1872\n"
+"-0.009621,-0.999954,0.000000,0.000000,1887\n"
+"-0.009451,-0.999955,0.000000,0.000000,1902\n"
+"-0.009280,-0.999957,0.000000,0.000000,1917\n"
+"-0.009107,-0.999959,0.000000,0.000000,1932\n"
+"-0.008934,-0.999960,0.000000,0.000000,1947\n"
+"-0.008759,-0.999962,0.000000,0.000000,1962\n"
+"-0.008582,-0.999963,0.000000,0.000000,1977\n"
+"-0.008405,-0.999965,0.000000,0.000000,1992\n"
+"-0.008225,-0.999966,0.000000,0.000000,2007\n"
+"-0.008045,-0.999968,0.000000,0.000000,2022\n"
+"-0.007863,-0.999969,0.000000,0.000000,2037\n"
+"-0.007679,-0.999971,0.000000,0.000000,2052\n"
+"-0.007494,-0.999972,0.000000,0.000000,2067\n"
+"-0.007308,-0.999973,0.000000,0.000000,2081\n"
+"-0.007120,-0.999975,0.000000,0.000000,2095\n"
+"-0.006931,-0.999976,0.000000,0.000000,2109\n"
+"-0.006740,-0.999977,0.000000,0.000000,2123\n"
+"-0.006547,-0.999979,0.000000,0.000000,2137\n"
+"-0.006353,-0.999980,0.000000,0.000000,2151\n"
+"-0.006157,-0.999981,0.000000,0.000000,2165\n"
+"-0.005960,-0.999982,0.000000,0.000000,2179\n"
+"-0.005761,-0.999983,0.000000,0.000000,2193\n"
+"-0.005560,-0.999985,0.000000,0.000000,2207\n"
+"-0.005358,-0.999986,0.000000,0.000000,2221\n"
+"-0.005154,-0.999987,0.000000,0.000000,2235\n"
+"-0.004948,-0.999988,0.000000,0.000000,2249\n"
+"-0.004741,-0.999989,0.000000,0.000000,2263\n"
+"-0.004531,-0.999990,0.000000,0.000000,2277\n"
+"-0.004320,-0.999991,0.000000,0.000000,2291\n"
+"-0.004107,-0.999992,0.000000,0.000000,2305\n"
+"-0.003893,-0.999992,0.000000,0.000000,2318\n"
+"-0.003676,-0.999993,0.000000,0.000000,2331\n"
+"-0.003458,-0.999994,0.000000,0.000000,2344\n"
+"-0.003237,-0.999995,0.000000,0.000000,2357\n"
+"-0.003015,-0.999995,0.000000,0.000000,2370\n"
+"-0.002791,-0.999996,0.000000,0.000000,2383\n"
+"-0.002565,-0.999997,0.000000,0.000000,2396\n"
+"-0.002336,-0.999997,0.000000,0.000000,2409\n"
+"-0.002106,-0.999998,0.000000,0.000000,2422\n"
+"-0.001874,-0.999998,0.000000,0.000000,2435\n"
+"-0.001639,-0.999999,0.000000,0.000000,2448\n"
+"-0.001403,-0.999999,0.000000,0.000000,2461\n"
+"-0.001164,-0.999999,0.000000,0.000000,2474\n"
+"-0.000923,-1.000000,0.000000,0.000000,2487\n"
+"-0.000680,-1.000000,0.000000,0.000000,2500\n"
+"-0.000435,-1.000000,0.000000,0.000000,2513\n"
+"-0.000187,-1.000000,0.000000,0.000000,2525\n"
+"0.000062,-1.000000,0.000000,0.000000,2537\n"
+"0.000315,-1.000000,0.000000,0.000000,2549\n"
+"0.000569,-1.000000,0.000000,0.000000,2561\n"
+"0.000826,-1.000000,0.000000,0.000000,2573\n"
+"0.001086,-0.999999,0.000000,0.000000,2585\n"
+"0.001347,-0.999999,0.000000,0.000000,2597\n"
+"0.001612,-0.999999,0.000000,0.000000,2609\n"
+"0.001879,-0.999998,0.000000,0.000000,2621\n"
+"0.002182,-0.999998,0.000000,0.000000,2636\n"
+"0.002523,-0.999997,0.000000,0.000000,2651\n"
+"0.002868,-0.999996,0.000000,0.000000,2666\n"
+"0.003218,-0.999995,0.000000,0.000000,2681\n"
+"0.003571,-0.999994,0.000000,0.000000,2696\n"
+"0.003929,-0.999992,0.000000,0.000000,2711\n"
+"0.004292,-0.999991,0.000000,0.000000,2725\n"
+"0.004659,-0.999989,0.000000,0.000000,2739\n"
+"0.005031,-0.999987,0.000000,0.000000,2753\n"
+"0.005408,-0.999985,0.000000,0.000000,2767\n"
+"0.005789,-0.999983,0.000000,0.000000,2781\n"
+"0.006175,-0.999981,0.000000,0.000000,2795\n"
+"0.006567,-0.999978,0.000000,0.000000,2809\n"
+"0.006963,-0.999976,0.000000,0.000000,2823\n"
+"0.007365,-0.999973,0.000000,0.000000,2837\n"
+"0.007772,-0.999970,0.000000,0.000000,2851\n"
+"0.008185,-0.999967,0.000000,0.000000,2865\n"
+"0.008603,-0.999963,0.000000,0.000000,2878\n"
+"0.009026,-0.999959,0.000000,0.000000,2891\n"
+"0.009456,-0.999955,0.000000,0.000000,2904\n"
+"0.009891,-0.999951,0.000000,0.000000,2917\n"
+"0.010332,-0.999947,0.000000,0.000000,2930\n"
+"0.010780,-0.999942,0.000000,0.000000,2943\n"
+"0.011234,-0.999937,0.000000,0.000000,2956\n"
+"0.011694,-0.999932,0.000000,0.000000,2969\n"
+"0.012161,-0.999926,0.000000,0.000000,2982\n"
+"0.012634,-0.999920,0.000000,0.000000,2995\n"
+"0.013114,-0.999914,0.000000,0.000000,3008\n"
+"0.013601,-0.999907,0.000000,0.000000,3020\n"
+"0.014096,-0.999901,0.000000,0.000000,3032\n"
+"0.014597,-0.999893,0.000000,0.000000,3044\n"
+"0.015106,-0.999886,0.000000,0.000000,3056\n"
+"0.015623,-0.999878,0.000000,0.000000,3068\n"
+"0.016200,-0.999869,0.000000,0.000000,3082\n"
+"0.016841,-0.999858,0.000000,0.000000,3096\n"
+"0.017494,-0.999847,0.000000,0.000000,3110\n"
+"0.018159,-0.999835,0.000000,0.000000,3124\n"
+"0.018836,-0.999823,0.000000,0.000000,3138\n"
+"0.019526,-0.999809,0.000000,0.000000,3152\n"
+"0.020229,-0.999795,0.000000,0.000000,3166\n"
+"0.020946,-0.999781,0.000000,0.000000,3179\n"
+"0.021676,-0.999765,0.000000,0.000000,3192\n"
+"0.022422,-0.999749,0.000000,0.000000,3205\n"
+"0.023181,-0.999731,0.000000,0.000000,3218\n"
+"0.023957,-0.999713,0.000000,0.000000,3231\n"
+"0.024747,-0.999694,0.000000,0.000000,3244\n"
+"0.025554,-0.999673,0.000000,0.000000,3257\n"
+"0.026378,-0.999652,0.000000,0.000000,3270\n"
+"0.027219,-0.999629,0.000000,0.000000,3282\n"
+"0.028078,-0.999606,0.000000,0.000000,3294\n"
+"0.028955,-0.999581,0.000000,0.000000,3306\n"
+"0.029926,-0.999552,0.000000,0.000000,3320\n"
+"0.030998,-0.999519,0.000000,0.000000,3334\n"
+"0.032098,-0.999485,0.000000,0.000000,3348\n"
+"0.033226,-0.999448,0.000000,0.000000,3362\n"
+"0.034384,-0.999409,0.000000,0.000000,3375\n"
+"0.035574,-0.999367,0.000000,0.000000,3388\n"
+"0.036795,-0.999323,0.000000,0.000000,3401\n"
+"0.038051,-0.999276,0.000000,0.000000,3414\n"
+"0.039341,-0.999226,0.000000,0.000000,3427\n"
+"0.040668,-0.999173,0.000000,0.000000,3439\n"
+"0.042033,-0.999116,0.000000,0.000000,3451\n"
+"0.043438,-0.999056,0.000000,0.000000,3463\n"
+"0.044990,-0.998987,0.000000,0.000000,3477\n"
+"0.005635,-0.999984,0.000000,0.000000,3490\n"
+"0.017053,-0.999855,0.000000,0.000000,3503\n"
+"0.028668,-0.999589,0.000000,0.000000,3516\n"
+"0.040482,-0.999180,0.000000,0.000000,3529\n"
+"0.052494,-0.998621,0.000000,0.000000,3542\n"
+"0.064703,-0.997905,0.000000,0.000000,3555\n"
+"0.077109,-0.997023,0.000000,0.000000,3568\n"
+"0.089712,-0.995968,0.000000,0.000000,3581\n"
+"0.102509,-0.994732,0.000000,0.000000,3593\n"
+"0.115498,-0.993308,0.000000,0.000000,3605\n"
+"0.128677,-0.991687,0.000000,0.000000,3617\n"
+"0.142043,-0.989860,0.000000,0.000000,3629\n"
+"0.156446,-0.987687,0.000000,0.000000,3643\n"
+"0.171915,-0.985112,0.000000,0.000000,3656\n"
+"0.187605,-0.982245,0.000000,0.000000,3669\n"
+"0.203507,-0.979073,0.000000,0.000000,3682\n"
+"0.219613,-0.975587,0.000000,0.000000,3695\n"
+"0.235912,-0.971774,0.000000,0.000000,3708\n"
+"0.252394,-0.967624,0.000000,0.000000,3721\n"
+"0.269048,-0.963127,0.000000,0.000000,3734\n"
+"0.285859,-0.958272,0.000000,0.000000,3747\n"
+"0.302815,-0.953049,0.000000,0.000000,3760\n"
+"0.319899,-0.947452,0.000000,0.000000,3773\n"
+"0.337096,-0.941470,0.000000,0.000000,3785\n"
+"0.354389,-0.935098,0.000000,0.000000,3797\n"
+"0.371760,-0.928329,0.000000,0.000000,3809\n"
+"0.389188,-0.921158,0.000000,0.000000,3821\n"
+"0.406655,-0.913582,0.000000,0.000000,3833\n"
+"0.424140,-0.905597,0.000000,0.000000,3845\n"
+"0.441621,-0.897202,0.000000,0.000000,3857\n"
+"0.459076,-0.888397,0.000000,0.000000,3869\n"
+"0.477449,-0.878660,0.000000,0.000000,3882\n"
+"0.496701,-0.867922,0.000000,0.000000,3895\n"
+"0.515832,-0.856690,0.000000,0.000000,3908\n"
+"0.534811,-0.844971,0.000000,0.000000,3921\n"
+"0.553607,-0.832778,0.000000,0.000000,3934\n"
+"0.572188,-0.820123,0.000000,0.000000,3947\n"
+"0.590523,-0.807021,0.000000,0.000000,3960\n"
+"0.608584,-0.793489,0.000000,0.000000,3973\n"
+"0.626342,-0.779548,0.000000,0.000000,3986\n"
+"0.643770,-0.765219,0.000000,0.000000,3999\n"
+"0.660842,-0.750525,0.000000,0.000000,4012\n"
+"0.677535,-0.735490,0.000000,0.000000,4025\n"
+"0.693827,-0.720141,0.000000,0.000000,4038\n"
+"0.709699,-0.704505,0.000000,0.000000,4051\n"
+"0.725131,-0.688611,0.000000,0.000000,4064\n"
+"0.740110,-0.672486,0.000000,0.000000,4077\n"
+"0.753908,-0.656980,0.000000,0.000000,4089\n"
+"0.766582,-0.642146,0.000000,0.000000,4101\n"
+"0.778863,-0.627194,0.000000,0.000000,4113\n"
+"0.790747,-0.612144,0.000000,0.000000,4125\n"
+"0.802229,-0.597017,0.000000,0.000000,4137\n"
+"0.813307,-0.581835,0.000000,0.000000,4149\n"
+"0.823982,-0.566617,0.000000,0.000000,4161\n"
+"0.834253,-0.551382,0.000000,0.000000,4173\n"
+"0.844122,-0.536151,0.000000,0.000000,4186\n"
+"0.853593,-0.520940,0.000000,0.000000,4199\n"
+"0.862669,-0.505768,0.000000,0.000000,4212\n"
+"0.871356,-0.490651,0.000000,0.000000,4225\n"
+"0.879659,-0.475605,0.000000,0.000000,4238\n"
+"0.887585,-0.460644,0.000000,0.000000,4251\n"
+"0.895142,-0.445782,0.000000,0.000000,4264\n"
+"0.902337,-0.431032,0.000000,0.000000,4277\n"
+"0.909179,-0.416406,0.000000,0.000000,4290\n"
+"0.915677,-0.401915,0.000000,0.000000,4303\n"
+"0.921507,-0.388362,0.000000,0.000000,4315\n"
+"0.926729,-0.375731,0.000000,0.000000,4327\n"
+"0.931700,-0.363228,0.000000,0.000000,4339\n"
+"0.936429,-0.350857,0.000000,0.000000,4351\n"
+"0.940922,-0.338623,0.000000,0.000000,4363\n"
+"0.945186,-0.326531,0.000000,0.000000,4376\n"
+"0.949229,-0.314585,0.000000,0.000000,4389\n"
+"0.953058,-0.302787,0.000000,0.000000,4402\n"
+"0.956680,-0.291140,0.000000,0.000000,4415\n"
+"0.960103,-0.279648,0.000000,0.000000,4428\n"
+"0.963332,-0.268311,0.000000,0.000000,4441\n"
+"0.966376,-0.257133,0.000000,0.000000,4454\n"
+"0.969241,-0.246113,0.000000,0.000000,4467\n"
+"0.971934,-0.235253,0.000000,0.000000,4480\n"
+"0.974462,-0.224554,0.000000,0.000000,4494\n"
+"0.976830,-0.214015,0.000000,0.000000,4508\n"
+"0.978912,-0.204282,0.000000,0.000000,4520\n"
+"0.980739,-0.195325,0.000000,0.000000,4532\n"
+"0.982457,-0.186491,0.000000,0.000000,4544\n"
+"0.984070,-0.177780,0.000000,0.000000,4556\n"
+"0.985583,-0.169191,0.000000,0.000000,4568\n"
+"0.987000,-0.160723,0.000000,0.000000,4581\n"
+"0.988323,-0.152376,0.000000,0.000000,4594\n"
+"0.989556,-0.144150,0.000000,0.000000,4607\n"
+"0.990703,-0.136042,0.000000,0.000000,4620\n"
+"0.991767,-0.128052,0.000000,0.000000,4633\n"
+"0.992752,-0.120180,0.000000,0.000000,4646\n"
+"0.993660,-0.112423,0.000000,0.000000,4659\n"
+"0.994495,-0.104782,0.000000,0.000000,4672\n"
+"0.995260,-0.097254,0.000000,0.000000,4685\n"
+"0.995956,-0.089839,0.000000,0.000000,4698\n"
+"0.996588,-0.082535,0.000000,0.000000,4712\n"
+"0.997158,-0.075340,0.000000,0.000000,4726\n"
+"0.997668,-0.068255,0.000000,0.000000,4740\n"
+"0.998121,-0.061276,0.000000,0.000000,4754\n"
+"0.998519,-0.054403,0.000000,0.000000,4768\n"
+"0.998842,-0.048114,0.000000,0.000000,4780\n"
+"0.999101,-0.042388,0.000000,0.000000,4792\n"
+"0.999325,-0.036736,0.000000,0.000000,4804\n"
+"0.999514,-0.031159,0.000000,0.000000,4816\n"
+"0.999671,-0.025654,0.000000,0.000000,4828\n"
+"0.999796,-0.020221,0.000000,0.000000,4840\n"
+"0.999890,-0.014859,0.000000,0.000000,4853\n"
+"0.999954,-0.009567,0.000000,0.000000,4866\n"
+"0.999991,-0.004344,0.000000,0.000000,4879\n"
+"1.000000,0.000810,0.000000,0.000000,4892\n"
+"0.999983,0.005897,0.000000,0.000000,4905\n"
+"0.999940,0.010918,0.000000,0.000000,4918\n"
+"0.999874,0.015874,0.000000,0.000000,4931\n"
+"0.999784,0.020764,0.000000,0.000000,4944\n"
+"0.999672,0.025592,0.000000,0.000000,4957\n"
+"0.999539,0.030356,0.000000,0.000000,4970\n"
+"0.999385,0.035059,0.000000,0.000000,4983\n"
+"0.999212,0.039700,0.000000,0.000000,4997\n"
+"0.999019,0.044282,0.000000,0.000000,5011\n"
+"0.998808,0.048804,0.000000,0.000000,5025\n"
+"0.998580,0.053269,0.000000,0.000000,5039\n"
+"0.998335,0.057675,0.000000,0.000000,5053\n"
+"0.998075,0.062025,0.000000,0.000000,5067\n"
+"0.997458,0.071257,0.000000,0.000000,5079\n"
+"0.995196,0.097898,0.000000,0.000000,5091\n"
+"0.991972,0.126456,0.000000,0.000000,5104\n"
+"0.987609,0.156935,0.000000,0.000000,5117\n"
+"0.982214,0.187764,0.000000,0.000000,5130\n"
+"0.975757,0.218856,0.000000,0.000000,5143\n"
+"0.968216,0.250116,0.000000,0.000000,5156\n"
+"0.959577,0.281447,0.000000,0.000000,5169\n"
+"0.949837,0.312744,0.000000,0.000000,5182\n"
+"0.939005,0.343903,0.000000,0.000000,5195\n"
+"0.927098,0.374818,0.000000,0.000000,5208\n"
+"0.914146,0.405384,0.000000,0.000000,5221\n"
+"0.900189,0.435499,0.000000,0.000000,5234\n"
+"0.885276,0.465065,0.000000,0.000000,5247\n"
+"0.869467,0.493991,0.000000,0.000000,5260\n"
+"0.852827,0.522193,0.000000,0.000000,5273\n"
+"0.835431,0.549595,0.000000,0.000000,5286\n"
+"0.817357,0.576132,0.000000,0.000000,5299\n"
+"0.798687,0.601747,0.000000,0.000000,5312\n"
+"0.779505,0.626396,0.000000,0.000000,5325\n"
+"0.759898,0.650043,0.000000,0.000000,5338\n"
+"0.740952,0.671558,0.000000,0.000000,5350\n"
+"0.722784,0.691074,0.000000,0.000000,5362\n"
+"0.704461,0.709743,0.000000,0.000000,5374\n"
+"0.686038,0.727565,0.000000,0.000000,5386\n"
+"0.667570,0.744547,0.000000,0.000000,5399\n"
+"0.649104,0.760699,0.000000,0.000000,5412\n"
+"0.630688,0.776036,0.000000,0.000000,5425\n"
+"0.612364,0.790576,0.000000,0.000000,5438\n"
+"0.594170,0.804340,0.000000,0.000000,5451\n"
+"0.576140,0.817351,0.000000,0.000000,5464\n"
+"0.559292,0.828971,0.000000,0.000000,5476\n"
+"0.543615,0.839335,0.000000,0.000000,5488\n"
+"0.528130,0.849163,0.000000,0.000000,5500\n"
+"0.512853,0.858476,0.000000,0.000000,5513\n"
+"0.497796,0.867294,0.000000,0.000000,5526\n"
+"0.482969,0.875637,0.000000,0.000000,5539\n"
+"0.468383,0.883525,0.000000,0.000000,5552\n"
+"0.454046,0.890978,0.000000,0.000000,5565\n"
+"0.439962,0.898016,0.000000,0.000000,5578\n"
+"0.426138,0.904658,0.000000,0.000000,5592\n"
+"0.412576,0.910923,0.000000,0.000000,5606\n"
+"0.400102,0.916471,0.000000,0.000000,5618\n"
+"0.388670,0.921377,0.000000,0.000000,5630\n"
+"0.377443,0.926033,0.000000,0.000000,5642\n"
+"0.366420,0.930450,0.000000,0.000000,5655\n"
+"0.355600,0.934638,0.000000,0.000000,5668\n"
+"0.344982,0.938609,0.000000,0.000000,5681\n"
+"0.334566,0.942372,0.000000,0.000000,5694\n"
+"0.324348,0.945938,0.000000,0.000000,5707\n"
+"0.314328,0.949314,0.000000,0.000000,5720\n"
+"0.304504,0.952511,0.000000,0.000000,5734\n"
+"0.294871,0.955537,0.000000,0.000000,5748\n"
+"0.285429,0.958400,0.000000,0.000000,5762\n"
+"0.276175,0.961107,0.000000,0.000000,5776\n"
+"0.267746,0.963490,0.000000,0.000000,5788\n"
+"0.260105,0.965580,0.000000,0.000000,5800\n"
+"0.252595,0.967572,0.000000,0.000000,5812\n"
+"0.245215,0.969469,0.000000,0.000000,5824\n"
+"0.237962,0.971274,0.000000,0.000000,5837\n"
+"0.230835,0.972993,0.000000,0.000000,5850\n"
+"0.223831,0.974628,0.000000,0.000000,5863\n"
+"0.216948,0.976183,0.000000,0.000000,5876\n"
+"0.210184,0.977662,0.000000,0.000000,5889\n"
+"0.203538,0.979067,0.000000,0.000000,5902\n"
+"0.197006,0.980402,0.000000,0.000000,5915\n"
+"0.190587,0.981670,0.000000,0.000000,5928\n"
+"0.184279,0.982874,0.000000,0.000000,5942\n"
+"0.178080,0.984016,0.000000,0.000000,5956\n"
+"0.171987,0.985099,0.000000,0.000000,5970\n"
+"0.165999,0.986126,0.000000,0.000000,5984\n"
+"0.160114,0.987099,0.000000,0.000000,5998\n"
+"0.154329,0.988019,0.000000,0.000000,6012\n"
+"0.148643,0.988891,0.000000,0.000000,6026\n"
+"0.143516,0.989648,0.000000,0.000000,6038\n"
+"0.138924,0.990303,0.000000,0.000000,6050\n"
+"0.134397,0.990928,0.000000,0.000000,6062\n"
+"0.129934,0.991523,0.000000,0.000000,6074\n"
+"0.125533,0.992089,0.000000,0.000000,6086\n"
+"0.121194,0.992629,0.000000,0.000000,6099\n"
+"0.116916,0.993142,0.000000,0.000000,6112\n"
+"0.112697,0.993629,0.000000,0.000000,6125\n"
+"0.108537,0.994092,0.000000,0.000000,6138\n"
+"0.104434,0.994532,0.000000,0.000000,6151\n"
+"0.100388,0.994948,0.000000,0.000000,6164\n"
+"0.096397,0.995343,0.000000,0.000000,6177\n"
+"0.092461,0.995716,0.000000,0.000000,6190\n"
+"0.088579,0.996069,0.000000,0.000000,6203\n"
+"0.084750,0.996402,0.000000,0.000000,6216\n"
+"0.080972,0.996716,0.000000,0.000000,6229\n"
+"0.077246,0.997012,0.000000,0.000000,6243\n"
+"0.073570,0.997290,0.000000,0.000000,6257\n"
+"0.069943,0.997551,0.000000,0.000000,6271\n"
+"0.066364,0.997795,0.000000,0.000000,6285\n"
+"0.062833,0.998024,0.000000,0.000000,6299\n"
+"0.059349,0.998237,0.000000,0.000000,6313\n"
+"0.055911,0.998436,0.000000,0.000000,6327\n"
+"0.052518,0.998620,0.000000,0.000000,6341\n"
+"0.049170,0.998790,0.000000,0.000000,6355\n"
+"0.045865,0.998948,0.000000,0.000000,6369\n"
+"0.042603,0.999092,0.000000,0.000000,6384\n"
+"0.039383,0.999224,0.000000,0.000000,6399\n"
+"0.036205,0.999344,0.000000,0.000000,6414\n"
+"0.033068,0.999453,0.000000,0.000000,6429\n"
+"0.029971,0.999551,0.000000,0.000000,6444\n"
+"0.026913,0.999638,0.000000,0.000000,6459\n"
+"0.024194,0.999707,0.000000,0.000000,6471\n"
+"0.021803,0.999762,0.000000,0.000000,6483\n"
+"0.019437,0.999811,0.000000,0.000000,6495\n"
+"0.017094,0.999854,0.000000,0.000000,6507\n"
+"0.014775,0.999891,0.000000,0.000000,6519\n"
+"0.012478,0.999922,0.000000,0.000000,6531\n"
+"0.010205,0.999948,0.000000,0.000000,6543\n"
+"0.007954,0.999968,0.000000,0.000000,6555\n"
+"0.005726,0.999984,0.000000,0.000000,6568\n"
+"0.003519,0.999994,0.000000,0.000000,6581\n"
+"0.001334,0.999999,0.000000,0.000000,6594\n"
+"-0.000830,1.000000,0.000000,0.000000,6607\n"
+"-0.002972,0.999996,0.000000,0.000000,6620\n"
+"-0.005094,0.999987,0.000000,0.000000,6633\n"
+"-0.007195,0.999974,0.000000,0.000000,6646\n"
+"-0.009276,0.999957,0.000000,0.000000,6659\n"
+"-0.011337,0.999936,0.000000,0.000000,6672\n"
+"-0.013379,0.999911,0.000000,0.000000,6685\n"
+"-0.015400,0.999881,0.000000,0.000000,6698\n"
+"-0.017403,0.999849,0.000000,0.000000,6711\n"
+"-0.019386,0.999812,0.000000,0.000000,6724\n"
+"-0.021351,0.999772,0.000000,0.000000,6737\n"
+"-0.023298,0.999729,0.000000,0.000000,6750\n"
+"-0.025226,0.999682,0.000000,0.000000,6763\n"
+"-0.027136,0.999632,0.000000,0.000000,6777\n"
+"-0.029028,0.999579,0.000000,0.000000,6791\n"
+"-0.030903,0.999522,0.000000,0.000000,6805\n"
+"-0.032760,0.999463,0.000000,0.000000,6819\n"
+"-0.034600,0.999401,0.000000,0.000000,6833\n"
+"-0.036424,0.999336,0.000000,0.000000,6847\n"
+"-0.038230,0.999269,0.000000,0.000000,6861\n"
+"-0.040020,0.999199,0.000000,0.000000,6875\n"
+"-0.041794,0.999126,0.000000,0.000000,6889\n"
+"-0.043552,0.999051,0.000000,0.000000,6903\n"
+"-0.045294,0.998974,0.000000,0.000000,6917\n"
+"-0.047021,0.998894,0.000000,0.000000,6931\n"
+"-0.048731,0.998812,0.000000,0.000000,6945\n"
+"-0.050427,0.998728,0.000000,0.000000,6959\n"
+"-0.052107,0.998641,0.000000,0.000000,6973\n"
+"-0.053773,0.998553,0.000000,0.000000,6988\n"
+"-0.055424,0.998463,0.000000,0.000000,7003\n"
+"-0.057060,0.998371,0.000000,0.000000,7018\n"
+"-0.058682,0.998277,0.000000,0.000000,7033\n"
+"-0.060290,0.998181,0.000000,0.000000,7048\n"
+"-0.061884,0.998083,0.000000,0.000000,7063\n"
+"-0.063464,0.997984,0.000000,0.000000,7078\n"
+"-0.065030,0.997883,0.000000,0.000000,7093\n"
+"-0.066583,0.997781,0.000000,0.000000,7108\n"
+"-0.068122,0.997677,0.000000,0.000000,7123\n"
+"-0.069648,0.997572,0.000000,0.000000,7138\n"
+"-0.071162,0.997465,0.000000,0.000000,7153\n"
+"-0.072662,0.997357,0.000000,0.000000,7168\n"
+"-0.074150,0.997247,0.000000,0.000000,7183\n"
+"-0.075625,0.997136,0.000000,0.000000,7198\n"
+"-0.077087,0.997024,0.000000,0.000000,7214\n"
+"-0.078538,0.996911,0.000000,0.000000,7230\n"
+"-0.079976,0.996797,0.000000,0.000000,7246\n"
+"-0.081402,0.996681,0.000000,0.000000,7262\n"
+"-0.082817,0.996565,0.000000,0.000000,7278\n"
+"-0.084219,0.996447,0.000000,0.000000,7294\n"
+"-0.085610,0.996329,0.000000,0.000000,7310\n"
+"-0.086990,0.996209,0.000000,0.000000,7326";

  /** Creates a new JoystickPlayback. */
  public JoystickPlayback(SwerveDrive swerve, String filename, int mult) {
    this.swerve = swerve;
    this.filename = filename;
    this.mult = mult;

    addRequirements(this.swerve);
  }

  /** Creates a new JoystickPlayback. */
  public JoystickPlayback(SwerveDrive swerve, String filename) {
    this(swerve, filename, 1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    outputs.clear();
    m_finished = false;

    startTime = System.currentTimeMillis();
    playbackTime = 0;
    lastIndex    = 0;
    // try {
    //   input = new Scanner(new File("/home/lvuser/autos/" + filename));

    //   String line = "";
    //   while (input.hasNextLine()) {
    //     line = input.nextLine();

    //     if (line.isEmpty() || line.isBlank() || line.equals("\n")) {
    //       continue;
    //     }
      
    //     String[] values = line.split(",");

    //     var out = new TimedOutput();
    //     out.leftX  = Double.parseDouble(values[0]) * mult;
    //     out.leftY  = Double.parseDouble(values[1]);
    //     out.rightX = Double.parseDouble(values[2]);
    //     out.rightY = Double.parseDouble(values[3]);

    //     out.timedOffset = Long.parseLong(values[4]);

    //     outputs.add(out);
    //   }

    //   input.close();
    // } catch (FileNotFoundException e) {
    //   e.printStackTrace();
    // }

    String[] inputs = path.split("\n");
    for (String line : inputs) {
        String[] values = line.split(",");

        var out = new TimedOutput();
        out.leftX  = Double.parseDouble(values[0]) * mult / -1.68;
        out.leftY  = Double.parseDouble(values[1]) / -1.68;
        out.rightX = Double.parseDouble(values[2]);
        out.rightY = Double.parseDouble(values[3]);

        out.timedOffset = Long.parseLong(values[4]);

        outputs.add(out);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (counter == 0) {
      startTime = System.currentTimeMillis();
      playbackTime = 0;
    } else {
      playbackTime = System.currentTimeMillis() - startTime;
    }

    // skip to reasonable time frame
    // too tired to write comment: ask daniel thomas; it goes to the thing until it's bigger than the other thing
    {
      int i = lastIndex == 0 ? 1 : lastIndex;
      while (i < outputs.size() && outputs.get(i).timedOffset < playbackTime) {
        i++;
      }

      if (i >= outputs.size()) {
        m_finished = true; // ! kind of a hack
        return;
      }
      lastIndex = i;
    }

    TimedOutput lastOut = outputs.get(lastIndex - 1);
    TimedOutput out     = outputs.get(lastIndex);

    double deltaTime     = out.timedOffset - lastOut.timedOffset;
    double playbackDelta = playbackTime    - lastOut.timedOffset;

    double lerpLX = lastOut.leftX  + (out.leftX  - lastOut.leftX)  * (playbackDelta / deltaTime);
    double lerpLY = lastOut.leftY  + (out.leftY  - lastOut.leftY)  * (playbackDelta / deltaTime);
    double lerpRX = lastOut.rightX + (out.rightX - lastOut.rightX) * (playbackDelta / deltaTime);
    double lerpRY = lastOut.rightY + (out.rightY - lastOut.rightY) * (playbackDelta / deltaTime);

    // this.swerve.driveWithInput(new Translation2d(out.leftX,   out.leftY),
    //                              new Translation2d(out.rightX, out.rightY),
    //                              true);
    
    this.swerve.driveWithInput( new Translation2d(lerpLX, lerpLY),
                                new Translation2d(lerpRX, lerpRY),
                                true);
                             
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    input.close();
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
