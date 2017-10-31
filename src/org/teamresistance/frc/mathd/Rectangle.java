package org.teamresistance.frc.mathd;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;

public class Rectangle implements LiveWindowSendable {
	
	private ITable table;
	
	public Vector2d position;
	public Vector2d size;
	
	public Rectangle(double positionX, double positionY, double sizeX, double sizeY) {
		this(new Vector2d(positionX, positionY), new Vector2d(sizeX, sizeY));
	}
	
	public Rectangle(Vector2d position, Vector2d size) {
		this.position = position;
		this.size = size;
	}
	
	public Vector2d getCenter() {
		return position.add(size.div(2.0));
	}

	@Override
	public void initTable(ITable subtable) {
		table = subtable;
		updateTable();
	}

	@Override
	public ITable getTable() {
		return table;
	}

	@Override
	public String getSmartDashboardType() {
		return "Rectangle";
	}

	@Override
	public void updateTable() {
		ITable positionTable = table.getSubTable("Position");
		positionTable.putString("~TYPE~", position.getSmartDashboardType());
		position.initTable(positionTable);
		
		ITable sizeTable = table.getSubTable("Size");
		sizeTable.putString("~TYPE~", position.getSmartDashboardType());
		size.initTable(sizeTable);
	}

	@Override
	public void startLiveWindowMode() {
		
	}

	@Override
	public void stopLiveWindowMode() {
		
	}
}
