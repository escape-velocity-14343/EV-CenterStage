package org.firstinspires.ftc.teamcode.drivers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DashboardPacketTelemetry implements Telemetry {
    private TelemetryPacket currentPacket;
    public DashboardPacketTelemetry() {
        currentPacket = new TelemetryPacket();
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        return addData(caption, String.format(format, args));
    }

    @Override
    public Item addData(String caption, Object value) {
        currentPacket.put(caption, value);
        return null;
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        throw new UnsupportedOperationException();
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean removeItem(Item item) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void clearAll() {
        clear();
    }

    @Override
    public Object addAction(Runnable action) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean removeAction(Object token) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void speak(String text) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean update() {
        //FtcDashboard.getInstance().sendTelemetryPacket(currentPacket);

        currentPacket = new TelemetryPacket();
        return true;
    }

    @Override
    public void clear() {
        throw new UnsupportedOperationException();
    }

    @Override
    public int getMsTransmissionInterval() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setMsTransmissionInterval(int a) {
        throw new UnsupportedOperationException();
    }

    @Override
    public Log log() {
        return null;
        //throw new UnsupportedOperationException();
    }
    @Override
    public Line addLine() {
        return null;
    }

    @Override
    public Line addLine(String lineCaption) {
        currentPacket.addLine(lineCaption);
        return null;
    }

    @Override
    public boolean removeLine(Line line) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean isAutoClear() {
        return false;
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        throw new UnsupportedOperationException();
    }


    @Override
    public String getItemSeparator() {
        return null;
    }

    @Override
    public void setItemSeparator(String itemSeparator) {

    }

    @Override
    public String getCaptionValueSeparator() {
        return null;
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {

    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {

    }

    public TelemetryPacket renderOdo(double fx, double fy, double heading, double targetx, double targety) {
        Canvas field = currentPacket.fieldOverlay();
        int robotRadius = 8;
        field.strokeCircle(fx, fy, robotRadius);
        double arrowX = new Rotation2d(heading).getCos() * robotRadius, arrowY = new Rotation2d(heading).getSin() * robotRadius;
        double x1 = fx, y1 = fy;
        double x2 = fx + arrowX, y2 = fy+ arrowY;
        field.strokeLine(x1, y1, x2, y2);
        field.setFill("yellow");
        field.setStroke("yellow");
        field.fillCircle(targetx, targety, 2);
        return currentPacket;
    }
}
