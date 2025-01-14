/****** Object:  StoredProcedure [dbo].[InsertXmlData]    Script Date: 11/7/2024 3:13:28 PM ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
-- =============================================
-- Author:		SSR
-- Create date: 2024-07-12
-- Description:	Stores XML file in SQL
-- =============================================
CREATE PROCEDURE [dbo].[InsertXmlData]
    @XmlData XML
AS
BEGIN
    -- Variables for project details
    DECLARE @Title NVARCHAR(255);
    DECLARE @Description NVARCHAR(MAX);
    DECLARE @Organization NVARCHAR(255);
    DECLARE @Members NVARCHAR(MAX);

    -- Variables for experiment details
    DECLARE @ExpTitle NVARCHAR(255);
    DECLARE @ExpDescription NVARCHAR(MAX);
    DECLARE @StartTime DATETIME;
    DECLARE @EndTime DATETIME;

    -- Variables to store IDs
    DECLARE @ProjectID INT;

    -- Extract project details from XML
    SELECT 
        @Title = Project.value('(title)[1]', 'NVARCHAR(255)'),
        @Description = Project.value('(description)[1]', 'NVARCHAR(MAX)'),
        @Organization = Project.value('(organization)[1]', 'NVARCHAR(255)'),
        @Members = Project.value('(members)[1]', 'NVARCHAR(MAX)')
    FROM @XmlData.nodes('/metadata/project') AS T(Project);

    -- Check if the project already exists
    SET @ProjectID = (
        SELECT ProjectID
        FROM Projects
        WHERE Title = @Title
    );

    -- If the project does not exist, insert it and get the new ProjectID
    IF @ProjectID IS NULL
    BEGIN
        INSERT INTO Projects (Title, Description, Organization, Members)
        VALUES (@Title, @Description, @Organization, @Members);
        
        -- Get the newly inserted ProjectID
        SET @ProjectID = SCOPE_IDENTITY();
    END

    -- Create a temporary table to store experiment data
    CREATE TABLE #Experiments (
        Title NVARCHAR(255),
        Description NVARCHAR(MAX),
        StartTime DATETIME,
        EndTime DATETIME
    );

    -- Insert experiment data into the temporary table
    INSERT INTO #Experiments (Title, Description, StartTime, EndTime)
    SELECT 
        Experiment.value('(title)[1]', 'NVARCHAR(255)'),
        Experiment.value('(description)[1]', 'NVARCHAR(MAX)'),
        CASE 
            WHEN ISNULL(Experiment.value('(starttime)[1]', 'NVARCHAR(255)'), '') = '' 
            THEN GETDATE() 
            ELSE Experiment.value('(starttime)[1]', 'DATETIME') 
        END,
        NULLIF(Experiment.value('(endtime)[1]', 'NVARCHAR(255)'), '') 
    FROM @XmlData.nodes('/metadata/experiment') AS T(Experiment);

    -- Insert experiments into the Experiments table
    INSERT INTO Experiments (ProjectID, Title, Description, StartTime, EndTime)
    SELECT 
        @ProjectID,
        Title,
        Description,
        StartTime,
        EndTime
    FROM #Experiments;

    -- Drop the temporary table
    DROP TABLE #Experiments;

    -- Insert XML data into ProjectXMLLogs table
    INSERT INTO ProjectXMLLogs (ProjectID, XMLData)
    VALUES (@ProjectID, @XmlData);
END;
