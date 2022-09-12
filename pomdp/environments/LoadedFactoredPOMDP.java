package pomdp.environments;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.Collection;
import java.util.Map;
import java.util.TreeMap;
import java.util.Vector;

import pomdp.environments.FactoredPOMDP.BeliefType;
import pomdp.utilities.InvalidModelFileFormatException;

public class LoadedFactoredPOMDP extends FactoredPOMDP {

	private int m_cTrueVariables;
	private Vector<String> m_vVariableNames, m_vObservationNames, m_vActionNames;
	private Vector<Vector<String>> m_vVariableValues;
	private Vector<Vector<Double>> m_vInitialStateProbabilities;
	private Map<String, Map<String, DecisionTree>> m_mTransitions;
	private Map<String, DecisionTree> m_mObservations;
	private Map<String, DecisionTree> m_mRewards;
	private Vector<Vector<Integer>> m_vMapTrueVariablesToBinary;
	
	
	public LoadedFactoredPOMDP( BeliefType btFactored ){
		super( 0, 0, 0, btFactored, false, true );
		m_vVariableNames = new Vector<String>();
		m_vObservationNames = new Vector<String>();
		m_vActionNames = new Vector<String>();
		m_vVariableValues = new Vector<Vector<String>>();
		m_vInitialStateProbabilities = new Vector<Vector<Double>>();
		m_mTransitions = new TreeMap<String, Map<String,DecisionTree>>();
		m_mObservations = new TreeMap<String, DecisionTree>();
		m_mRewards = new TreeMap<String, DecisionTree>();
		m_vMapTrueVariablesToBinary = new Vector<Vector<Integer>>();
	}
	
	public void load( String sFileName ) throws IOException, InvalidModelFileFormatException{
		String sToken = "";
		FileInputStream fis = new FileInputStream( sFileName );
		while( fis.available() > 0 ){
			sToken = nextToken( fis );
			if( sToken.toLowerCase().equals( "(variables" ) )
				readVariables( fis );
			else if( sToken.toLowerCase().equals( "(observations" ) )
				readObservations( fis );
			else if( sToken.toLowerCase().equals( "init" ) )
				readInitialState( fis );
			else if( sToken.startsWith( "//" ) )
				readLine( fis );
			else if( sToken.toLowerCase().equals( "action" ) )
				readAction( fis );
			else if( sToken.toLowerCase().equals( "name" ) )
				m_sName = nextToken( fis );
		}

		initADDs();
		
		double dMdpAdr = computeMDPAverageDiscountedReward( 200, 250 );
		System.out.println( "MDP ADR = " + dMdpAdr );
	}
	
	public String getActionName( int iAction ){
		return m_vActionNames.elementAt( iAction );
	}
	public int getActionIndex( String sAction ){
		return m_vActionNames.indexOf( sAction );
	}
	
	private void readAction( FileInputStream fis ) throws IOException, InvalidModelFileFormatException{
		String sActionName = nextToken( fis );
		TreeMap<String, DecisionTree> mTransitions = new TreeMap<String, DecisionTree>();
		DecisionTree dt = null;
		String sType = "";

		m_cActions++;
		m_mTransitions.put( sActionName, mTransitions );
		m_vActionNames.add( sActionName );
		
		while( !sType.equals( "endaction" ) ){
			if( sType.trim().length() > 0 ){
				if( sType.equals( "observe" ) ){
					dt = readObservation( fis );
					m_mObservations.put( sActionName, dt );
				}
				else if( sType.equals( "cost" ) ){
					dt = readCost( readBetweenParanthese( fis ) );
					dt.negateValues();
					m_mRewards.put( sActionName, dt );
				}
				else{
					dt = readTransition( readBetweenParanthese( fis ) );
					mTransitions.put( sType, dt );
				}
				if( dt != null )
					System.out.println( sActionName + " " + sType + " " + dt.toString() );
			}
			sType = nextToken( fis );
		}		
	}


	private DecisionTree construct( String sData ){
		DecisionTree dt = new DecisionTree(), dtChild = null;
		String sChild = "", sVarName = "", sValName = "", sValue = "";
		int idx = sData.indexOf( '(', 1 ), idxChild = 0;
		if( idx == -1 ){ //leaf
			sValue = sData.substring( 1, sData.length() - 1 );
			dt.setValue( Double.parseDouble( sValue ) );
		}
		else{
			sVarName = sData.substring( 1, idx - 1 ).trim();
			sData = sData.substring( idx ).trim();
			dt.setVarName( sVarName );
			while( sData.length() > 1 ){
				sChild = readBetweenParanthese( sData, 0 );
				sData = sData.substring( sChild.length() ).trim();
				idxChild = sChild.indexOf( '(', 1 );
				sValName = sChild.substring( 1, idxChild - 1 ).trim();
				sChild = readBetweenParanthese( sChild, idxChild );
				dtChild = construct( sChild );
				dt.addChild( sValName, dtChild );
			}
		}
		
		return dt;
	}

	private DecisionTree readTransition( String sTransition ){
		if( sTransition.startsWith( "(SAME" ) )
			return null;
		return construct( sTransition );
	}



	private DecisionTree readCost( String sCost ) {
		return construct( sCost );
	}



	private DecisionTree readObservation( FileInputStream fis ) throws IOException, InvalidModelFileFormatException {
		String sObservationData = "";
		String sToken = ""; //get rid of "ObserveResult" - thus assuming a single observation variable
		while( !sToken.equals( "ObserveResult" ) )
			sToken = nextToken( fis );
		sToken = nextToken( fis );
		while( !sToken.equals( "endobserve" ) ){
			sObservationData += sToken + " ";
			sToken = nextToken( fis );
		}
		return construct( sObservationData );
	}



	private String readLine(FileInputStream fis)  throws IOException{
		char c = '\0';
		String sLine = "";
		do{
			c = (char)fis.read();
			sLine += c;
		}while( c != '\n' );
		return sLine.trim();
	}

	private String readBetweenParanthese( FileInputStream fis )  throws IOException{
		char c = '\0';
		String sLine = "";
		int cParanthese = 0;
		do{
			c = (char)fis.read();
			if( c == '(' ){
				sLine += c;
				cParanthese++;
			}
			if( c == ')' )
				cParanthese--;
		}while( cParanthese == 0 );
		do{
			c = (char)fis.read();
			if( c == '(' )
				cParanthese++;
			if( c == ')' )
				cParanthese--;
			sLine += c;
		}while( cParanthese > 0 );
		return sLine.trim();
	}

	private String readBetweenParanthese( String s, int iStart ){
		char c = '\0';
		String sLine = "";
		int cParanthese = 0;
		int i = iStart;
		do{
			c = s.charAt( i );
			if( c == '(' )
				cParanthese++;
			if( c == ')' )
				cParanthese--;
			sLine += c;
			i++;
		}while( cParanthese > 0 );
		return sLine.trim();
	}


	private void readVariables( FileInputStream fis ) throws IOException, InvalidModelFileFormatException{
		char c = '\0';
		String sToken = "";
		boolean bDone = false;
		int cValues = 0;
		m_cStateVariables = 0;
		Vector<Integer> vBinaries = null;
		while( c != ')' ){
			c = (char)fis.read();
			if( c == '(' ){
				sToken = nextToken( fis );
				m_vVariableNames.add( sToken );
				m_vVariableValues.add( new Vector<String>() );
				m_vInitialStateProbabilities.add( new Vector<Double>() );
				m_cTrueVariables++;
				vBinaries = new Vector<Integer>();
				m_vMapTrueVariablesToBinary.add( vBinaries );
				bDone = false;
				cValues = 0;
				while( !bDone ){
					sToken = nextToken( fis );
					if( sToken.charAt( sToken.length() - 1 ) == ')' ){
						bDone = true;
						sToken = sToken.substring( 0, sToken.length() - 1 );
					}
					cValues++;
					m_vVariableValues.elementAt( m_cTrueVariables - 1 ).add( sToken );
					m_vInitialStateProbabilities.elementAt( m_cTrueVariables - 1 ).add( 0.0 );
				}
				while( cValues > 1 ){
					vBinaries.add( m_cStateVariables );
					m_cStateVariables++;
					cValues = ( cValues + 1 ) / 2;
				}
			}
		}
		
		System.out.println( "Variables: " + m_cTrueVariables );
		for( int i = 0 ; i < m_cTrueVariables ; i++ ){
			System.out.print( m_vVariableNames.elementAt( i ) );
			for( int j = 0 ; j < m_vVariableValues.elementAt( i ).size() ; j++ ){
				System.out.print( " " + m_vVariableValues.elementAt( i ).elementAt( j ) );
			}
			System.out.println();
		}
	}
	private void readInitialState( FileInputStream fis ) throws IOException, InvalidModelFileFormatException{
		char c = '\0';
		String sToken = "", sVariableName = "", sValueString = "", sValueName = "", sProb = "";
		boolean bDone = false;
		int idx = 0, iValue = 0;
		
		sToken = nextToken( fis ); //remove the [*
		
		while( c != ']' ){
			c = (char)fis.read();
			if( c == '(' ){
				sVariableName = nextToken( fis );
				
				for( iValue = 0 ; iValue < getValuesCount( sVariableName ) ; iValue++ ){
					sValueString = nextToken( fis );
					if( sValueString.endsWith( ")" ) ){
						sValueString = sValueString.substring( 1, sValueString.length() - 2 );
						idx = sValueString.indexOf( '(' );
						sValueName = sValueString.substring( 0, idx );
						sValueString = sValueString.substring( idx + 1 );
						if( sValueString.indexOf( ')' ) != -1 )
							sProb = sValueString.substring( 0, sValueString.indexOf( ')' ) );
						else
							sProb = sValueString;
					}
					else{
						sValueName = sValueString.substring( 1 );
						sProb = nextToken( fis );
						sProb = sProb.substring( 1, sProb.indexOf( ')' ) - 3 );
					}
					m_vInitialStateProbabilities.elementAt( getVariableIndex( sVariableName ) ).set( getValueIndex( sVariableName, sValueName ), Double.parseDouble( sProb ) );
				}
			}
		}
		
		System.out.println( "Initial values: " );
		for( int i = 0 ; i < m_cTrueVariables ; i++ ){
			System.out.print( m_vVariableNames.elementAt( i ) );
			for( int j = 0 ; j < m_vVariableValues.elementAt( i ).size() ; j++ ){
				System.out.print( " " + m_vVariableValues.elementAt( i ).elementAt( j ) + "=" + m_vInitialStateProbabilities.elementAt( i ).elementAt( j ) );
			}
			System.out.println();
		}
	}
	private int getValuesCount( String sVariableName ) {
		int idx = m_vVariableNames.indexOf( sVariableName );
		return m_vVariableValues.elementAt( idx ).size();
	}

	private int getValueIndex( String sVariableName, String sValueName ) {
		int idx = m_vVariableNames.indexOf( sVariableName );
		return m_vVariableValues.elementAt( idx ).indexOf( sValueName );
	}

	private String getValueName( String sVariableName, int iValue ){
		int idx = m_vVariableNames.indexOf( sVariableName );
		return m_vVariableValues.elementAt( idx ).elementAt( iValue );
	}
	
	private int getVariableIndex( String sVariableName ) {
		if( sVariableName.endsWith( "'" ) )
			sVariableName = sVariableName.substring( 0, sVariableName.length() - 1 ) ;
		int idx = m_vVariableNames.indexOf( sVariableName );
		return idx;
	}

	private void readObservations( FileInputStream fis ) throws IOException, InvalidModelFileFormatException{
		String sToken = nextToken( fis );
		boolean bDone = false;
		while( !bDone ){
			sToken = nextToken( fis );
			m_cObservations++;
			if( sToken.charAt( sToken.length() - 1 ) == ')' ){
				bDone = true;
				sToken = sToken.substring( 0, sToken.length() - 2 );
			}
			m_vObservationNames.add( sToken );
		}
		
		System.out.println( "Observations: " + m_cObservations );
		for( int i = 0 ; i < m_cObservations ; i++ ){
			System.out.print( " " + m_vObservationNames.elementAt( i ) );
		}
		System.out.println();
	}
	
	private boolean isAlphaNumeric( char c ){
		if( ( c >= 'a' ) && ( c <= 'z' ) )
			return true;
		if( ( c >= 'A' ) && ( c <= 'Z' ) )
			return true;
		if( ( c >= '0' ) && ( c <= '9' ) )
			return true;
		if( ( c == '_' ) || ( c == '-' ) || ( c == '+' ) || ( c == '.' ) || ( c == '(' ) || ( c == ')' ) || ( c == '[' ) || ( c == ']' ) || ( c == '*' ) || ( c == '/' ) )
			return true;
		return false;
	}
	
	private String nextToken( FileInputStream fis ) throws IOException, InvalidModelFileFormatException{
		String sToken = "";
		char c = '\0';
		while( isAlphaNumeric( c = (char)fis.read() ) )
			sToken += c;
		return sToken.trim();
	}
	
	
	@Override
	public boolean changingComponent(int component, int action, int observation) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public int[] getIndependentComponentVariables(int component) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public int getIndependentComponentsCount() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getInitialComponenetValueProbability(int component, int value) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected double getInitialVariableValueProbability(int variable, int value) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getInitialVariableValueProbability(int variable, boolean value) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public String getObservationName(int observation) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	protected String getObservationName() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	protected int[] getObservationRelevantVariables(int action) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	protected int[] getObservationRelevantVariablesMultiValue(int action) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	protected int getRealStateVariableCount() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected String getRealVariableName(int variable) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public int[] getRelevantComponents(int action, int observation) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public int[] getRelevantComponentsForComponent(int action, int component) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public int[] getRelevantVariables( int iAction ) {
		return getVariablesInDecisionTree( m_mTransitions.get( getActionName( iAction ) ).values() );
	}

	@Override
	public int[] getRelevantVariablesForComponent(int action, int component) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	protected int[] getRelevantVariablesMultiValue(int action, int variable) {
		// TODO Auto-generated method stub
		return null;
	}

	protected int[] getVariablesInDecisionTree( DecisionTree dt ){
		Vector<String> vVariables = dt.getVariables();
		Vector<Integer> vBinaryVariables = new Vector<Integer>();
		for( String sVariable : vVariables ){
			int iTrueVar = getVariableIndex( sVariable );
			vBinaryVariables.addAll( m_vMapTrueVariablesToBinary.elementAt( iTrueVar ) );
		}
		int[] aVariables = new int[vBinaryVariables.size()];
		for( int iBinaryVariable = 0 ; iBinaryVariable < vBinaryVariables.size() ; iBinaryVariable++ ){
			aVariables[iBinaryVariable] = vBinaryVariables.elementAt( iBinaryVariable );
		}
		return aVariables;
	}
	
	protected int[] getVariablesInDecisionTree( Collection<DecisionTree> vDT ){
		Vector<String> vVariables = new Vector<String>();
		for( DecisionTree dt : vDT ){
			if( dt != null ){
				Vector<String> vDTVariables = dt.getVariables();
				for( String sVariable : vDTVariables ){
					if( sVariable.endsWith( "'" ) )
						sVariable = sVariable.substring( 0, sVariable.length() - 1 ) ;
					if( !vVariables.contains( sVariable ) )
						vVariables.add( sVariable );
				}			
			}
		}
		Vector<Integer> vBinaryVariables = new Vector<Integer>();
		for( String sVariable : vVariables ){
			int iTrueVar = getVariableIndex( sVariable );
			vBinaryVariables.addAll( m_vMapTrueVariablesToBinary.elementAt( iTrueVar ) );
		}
		int[] aVariables = new int[vBinaryVariables.size()];
		for( int iBinaryVariable = 0 ; iBinaryVariable < vBinaryVariables.size() ; iBinaryVariable++ ){
			aVariables[iBinaryVariable] = vBinaryVariables.elementAt( iBinaryVariable );
		}
		return aVariables;
	}
	
	@Override
	protected int[] getRewardRelevantVariables( int iAction ) {
		return getVariablesInDecisionTree( m_mRewards.get( getActionName( iAction ) ) );

	}
	
	public boolean[] indexToState( int iStateIndex ){
		boolean[] abState = new boolean[m_cStateVariables];
		int idx = 0;
		for( idx = 0 ; idx < m_cStateVariables ; idx++ ){
			if( iStateIndex % 2 == 1 )
				abState[idx] = true;
			else
				abState[idx] = false;
			iStateIndex /= 2;
		}
		return abState;
	}

	public int getVariableValue( String sVariable, boolean[] abState ){
		int iVariable = getVariableIndex( sVariable );
		Vector<Integer> vBinaryVariables = m_vMapTrueVariablesToBinary.elementAt( iVariable );
		int iValue = 0, idx = 0;
		for( idx = vBinaryVariables.size() - 1 ; idx >= 0 ; idx-- ){
			iValue = iValue * 2;
			if( abState[vBinaryVariables.elementAt( idx )] )
				iValue += 1;
		}
		return iValue;
	}
	
	public double R( int iStartState, int iAction ){
		boolean[] abState = indexToState( iStartState );
		String sAction = getActionName( iAction );
		DecisionTree dtR = m_mRewards.get( sAction );
		String sVariable = "", sValue = "";
		int iVariableValue = 0;
		while( !dtR.isLeaf() ){
			sVariable = dtR.getVarName();
			iVariableValue = getVariableValue( sVariable, abState );
			sValue = getValueName( sVariable, iVariableValue );
			dtR = dtR.getChild( sValue );
		}
		return dtR.getValue();
	}
	
	@Override
	protected int[] getRewardRelevantVariablesMultiValue(int action) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	protected int getValueCount(int variable) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected String getValueName(int variable, int value) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	protected String getVariableName( int iVariable) {
		return m_vVariableNames.elementAt( iVariable );
	}

	@Override
	public double observationGivenRelevantVariables(int action,
			int observation, int[] aiRelevantVariables, boolean[] abValues) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected double observationGivenRelevantVariablesMultiValue(int action,
			int observation, int[] aiRelevantVariables, int[] aiValues) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected boolean relevantTransitionRealVariable(int action, int variable) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected boolean relevantTransitionVariable(int action, int variable) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public double rewardGivenRelevantVariables(int action,
			int[] aiRelevantVariables, boolean[] abValues) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected double rewardGivenRelevantVariablesMultiValue(int action,
			int[] aiRelevantVariables, int[] aiValues) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected double stateVariableObservation(int[] aiStateVariableIndexes,
			int action, boolean[] abStateVariableValues, int observation) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected double stateVariableTransition(int[] aiStateVariableIndexes,
			boolean[] abStateVariableValuesBefore, int action,
			boolean[] abStateVariableValuesAfter) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected double stateVariableTransition(int state, int action,
			int stateVariable) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double transitionGivenRelevantVariables(int action, int variable,
			boolean value, int[] aiRelevantVariables, boolean[] abValues) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double transitionGivenRelevantVariables(int action,
			int[] aiComponent, boolean[] abComponentValues,
			int[] aiRelevantVariables, boolean[] abRelevantValues) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected double transitionGivenRelevantVariablesMultiValue(int action,
			int variable, int value, int[] aiRelevantVariables, int[] atValues) {
		// TODO Auto-generated method stub
		return 0;
	}
	
	
	public static void main( String[] args ) throws IOException, InvalidModelFileFormatException{
		LoadedFactoredPOMDP p = new LoadedFactoredPOMDP( BeliefType.Factored );
		p.load( "SymbolicPerseus/PW2_2_2.pomdp" );
	}
	
	private class DecisionTree{
		private Map<String, DecisionTree> m_mChildren;
		private double m_dValue;
		private String m_sVarName;
		
		public DecisionTree(){
			m_mChildren = null;
			m_dValue = 0.0;
			m_sVarName = "";
		}
		public Vector<String> getVariables() {
			Vector<String> vVariables = new Vector<String>();
			if( m_mChildren != null ){
				vVariables.add( m_sVarName );
				for( DecisionTree dtChild : m_mChildren.values() ){
					Vector<String> vChildVariables = dtChild.getVariables();
					for( String sVar : vChildVariables ){
						if( !vVariables.contains( sVar ) )
							vVariables.add( sVar );
					}
				}
			}
			return vVariables;
		}
		public void negateValues() {
			if( m_dValue != 0.0 )
				m_dValue = m_dValue * -1.0;
			if( m_mChildren != null ){
				for( DecisionTree dtChild : m_mChildren.values() )
					dtChild.negateValues();
			}
			
		}
		public boolean isLeaf(){
			return m_mChildren == null;
		}
		public double getValue(){
			return m_dValue;
		}
		public DecisionTree getChild( String sValueName ){
			if( ( m_mChildren != null ) && m_mChildren.containsKey( sValueName ) )
				return m_mChildren.get( sValueName );
			return null;
		}
		public String getVarName(){
			return m_sVarName;
		}
		public void setValue( double dValue ){
			m_dValue = dValue;
		}
		public void setVarName( String sName ){
			m_sVarName = sName;
		}
		public void addChild( String sVarName, DecisionTree dtChild ){
			if( m_mChildren == null )
				m_mChildren = new TreeMap<String, DecisionTree>();
			m_mChildren.put( sVarName, dtChild );
		}
		public String toString(){
			String sExp = "(";
			if( isLeaf() ){
				sExp += m_dValue + ")";
			}
			else{
				sExp += m_sVarName + " ";
				for( Map.Entry<String, DecisionTree> eChild : m_mChildren.entrySet() ){
					sExp += "(" + eChild.getKey() + " " + eChild.getValue().toString() + ") ";
				}
				sExp = sExp.trim() + ")";
			}
			return sExp;
		}
	}

	@Override
	protected int stateToIndex(boolean[] abState) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected int stateToIndex(int[] aiStateVariableIndexes,
			boolean[] abStateVariableValues) {
		// TODO Auto-generated method stub
		return 0;
	}
}
